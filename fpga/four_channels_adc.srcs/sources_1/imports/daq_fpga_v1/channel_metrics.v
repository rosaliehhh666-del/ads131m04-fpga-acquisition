`timescale 1ns / 1ps
/*
 * Per-channel time-domain metrics over 2^LOG2_N samples.
 *
 * Outputs:
 *   mean    : signed ADC-code mean
 *   vpp     : max - min, in ADC codes
 *   rms_ac  : sqrt(E[x^2] - E[x]^2), in ADC codes
 *
 * The AC RMS form removes DC offset automatically.
 */
module channel_metrics #(
    parameter integer LOG2_N = 11
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    input  wire signed [23:0] sample,

    output reg  signed [23:0] mean,
    output reg         [24:0] vpp,
    output reg         [23:0] rms_ac,
    output reg                metrics_valid
);

localparam integer N = (1 << LOG2_N);

reg [LOG2_N-1:0] sample_count;

reg signed [47:0] sum_acc;
reg        [63:0] sumsq_acc;

reg signed [23:0] max_s;
reg signed [23:0] min_s;

wire signed [47:0] sample_ext =
    {{24{sample[23]}}, sample};

wire signed [47:0] sum_add =
    sum_acc + sample_ext;

wire signed [47:0] sq_signed =
    sample * sample;

wire [47:0] sq =
    sq_signed[47:0];

wire [63:0] sumsq_add =
    sumsq_acc + {{16{1'b0}}, sq};

wire signed [23:0] max_next =
    ((sample_count == 0) || (sample > max_s)) ? sample : max_s;

wire signed [23:0] min_next =
    ((sample_count == 0) || (sample < min_s)) ? sample : min_s;

wire signed [24:0] max_ext25 = {max_next[23], max_next};
wire signed [24:0] min_ext25 = {min_next[23], min_next};
wire signed [24:0] vpp_signed = max_ext25 - min_ext25;

wire signed [47:0] mean_wide =
    sum_add >>> LOG2_N;

wire signed [23:0] mean_next =
    mean_wide[23:0];

wire signed [47:0] mean_sq_signed =
    mean_next * mean_next;

wire [47:0] mean_sq =
    mean_sq_signed[47:0];

wire [63:0] mean_square_total =
    sumsq_add >> LOG2_N;

wire [63:0] mean_sq_ext =
    {{16{1'b0}}, mean_sq};

wire [63:0] variance =
    (mean_square_total >= mean_sq_ext) ?
    (mean_square_total - mean_sq_ext) : 64'd0;

reg        sqrt_start;
reg [63:0] sqrt_value;
wire [31:0] sqrt_result;
wire sqrt_busy;
wire sqrt_done;

isqrt_u64 u_sqrt (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (sqrt_start),
    .radicand (sqrt_value),
    .result   (sqrt_result),
    .busy     (sqrt_busy),
    .done     (sqrt_done)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_count  <= {LOG2_N{1'b0}};
        sum_acc       <= 48'sd0;
        sumsq_acc     <= 64'd0;
        max_s         <= 24'sd0;
        min_s         <= 24'sd0;

        mean          <= 24'sd0;
        vpp           <= 25'd0;
        rms_ac        <= 24'd0;
        metrics_valid <= 1'b0;

        sqrt_start    <= 1'b0;
        sqrt_value    <= 64'd0;
    end else begin
        sqrt_start    <= 1'b0;
        metrics_valid <= 1'b0;

        if (sqrt_done) begin
            rms_ac        <= sqrt_result[23:0];
            metrics_valid <= 1'b1;
        end

        if (sample_valid) begin
            if (sample_count == N-1) begin
                mean       <= mean_next;
                vpp        <= vpp_signed[24:0];
                sqrt_value <= variance;
                sqrt_start <= 1'b1;

                sample_count <= {LOG2_N{1'b0}};
                sum_acc      <= 48'sd0;
                sumsq_acc    <= 64'd0;
                max_s        <= 24'sd0;
                min_s        <= 24'sd0;
            end else begin
                sample_count <= sample_count + 1'b1;
                sum_acc      <= sum_add;
                sumsq_acc    <= sumsq_add;
                max_s        <= max_next;
                min_s        <= min_next;
            end
        end
    end
end

endmodule

`timescale 1ns / 1ps
`default_nettype none
/*
 * Non-overlapping windows of 2^LOG2_N signed 24-bit samples.
 * sample_valid is a sampling strobe, NOT a backpressured AXI interface.
 * Every sample_valid advances the acquisition window, including while a
 * preceding RMS is calculated or its output is waiting to be consumed.
 *
 * mean = floor(sum/N), vpp = max-min, rms_ac = exact integer AC RMS.
 * mean/vpp/rms_ac/window_id are published together, and held while
 * metrics_valid && !metrics_ready. window_id starts at 1.
 *
 * A completed window that cannot enter rms_ac_calc is explicitly dropped;
 * window_dropped pulses and its window_id is skipped. No samples from
 * different windows are joined to hide a drop. At LOG2_N=11 the calculator
 * is comfortably faster than even one sample per clk when the consumer
 * remains ready. Smaller windows/backpressure can cause reported drops.
 */
module channel_metrics_fix #(
    parameter integer LOG2_N = 11
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    input  wire signed [23:0] sample,
    output reg  signed [23:0] mean,
    output reg  [24:0]        vpp,
    output reg  [23:0]        rms_ac,
    output reg                metrics_valid,
    input  wire               metrics_ready,
    output reg  [31:0]        window_id,
    output wire               window_ready,
    output wire               calc_busy,
    output reg                window_dropped,
    output reg                arithmetic_error
);
    localparam integer SUM_W = 24 + LOG2_N;
    reg [LOG2_N-1:0] count;
    reg signed [SUM_W-1:0] sum_acc;
    reg [63:0] sumsq_acc;
    reg signed [23:0] max_sample, min_sample;
    reg [31:0] window_count;
    reg signed [23:0] pending_mean;
    reg [24:0] pending_vpp;
    reg [31:0] pending_id;

    wire signed [SUM_W-1:0] sample_ext =
        {{LOG2_N{sample[23]}}, sample};
    wire signed [SUM_W-1:0] sum_next = sum_acc + sample_ext;
    wire signed [47:0] sample_sq_signed = sample * sample;
    wire [63:0] sumsq_next = sumsq_acc + {16'd0, sample_sq_signed[47:0]};
    wire signed [23:0] max_next =
        ((count == 0) || (sample > max_sample)) ? sample : max_sample;
    wire signed [23:0] min_next =
        ((count == 0) || (sample < min_sample)) ? sample : min_sample;
    wire signed [24:0] max_ext = {max_next[23], max_next};
    wire signed [24:0] min_ext = {min_next[23], min_next};
    wire signed [24:0] vpp_next = max_ext - min_ext;
    wire signed [SUM_W-1:0] mean_wide = sum_next >>> LOG2_N;
    wire last_sample = &count;
    wire submit_window = sample_valid && last_sample;
    wire rms_valid;
    wire rms_ready = !metrics_valid || metrics_ready;
    wire [23:0] calculated_rms;
    wire rms_error;

    rms_ac_calc #(.LOG2_N(LOG2_N)) u_rms (
        .clk(clk), .rst_n(rst_n),
        .in_valid(submit_window), .in_ready(window_ready),
        .sum_in(sum_next), .sumsq_in(sumsq_next),
        .out_valid(rms_valid), .out_ready(rms_ready),
        .rms_ac(calculated_rms), .arithmetic_error(rms_error),
        .busy(calc_busy)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 0; sum_acc <= 0; sumsq_acc <= 0;
            max_sample <= 0; min_sample <= 0;
            window_count <= 0;
            pending_mean <= 0; pending_vpp <= 0; pending_id <= 0;
            mean <= 0; vpp <= 0; rms_ac <= 0; window_id <= 0;
            metrics_valid <= 0; window_dropped <= 0;
            arithmetic_error <= 0;
        end else begin
            window_dropped <= 1'b0;
            if (metrics_valid && metrics_ready)
                metrics_valid <= 1'b0;

            if (rms_valid && rms_ready) begin
                mean <= pending_mean;
                vpp <= pending_vpp;
                rms_ac <= calculated_rms;
                window_id <= pending_id;
                arithmetic_error <= rms_error;
                metrics_valid <= 1'b1;
            end

            if (sample_valid) begin
                if (last_sample) begin
                    // next values INCLUDE the Nth sample, not just N-1.
                    if (window_ready) begin
                        pending_mean <= mean_wide[23:0];
                        pending_vpp <= vpp_next[24:0];
                        pending_id <= window_count + 32'd1;
                    end else begin
                        window_dropped <= 1'b1;
                    end
                    window_count <= window_count + 32'd1;
                    count <= 0; sum_acc <= 0; sumsq_acc <= 0;
                    max_sample <= 0; min_sample <= 0;
                end else begin
                    count <= count + 1'b1;
                    sum_acc <= sum_next;
                    sumsq_acc <= sumsq_next;
                    max_sample <= max_next;
                    min_sample <= min_next;
                end
            end
        end
    end

    // synthesis translate_off
    initial begin
        if (LOG2_N < 1 || LOG2_N > 16)
            $fatal(1, "channel_metrics_fix requires LOG2_N in 1..16");
    end
    // synthesis translate_on
endmodule
`default_nettype wire

`timescale 1ns / 1ps
`default_nettype none
/*
 * Exact integer AC RMS from a completed 2^LOG2_N-sample window.
 * Inputs are sums of signed 24-bit ADC codes; sumsq_in is unsigned.
 *
 * rms = floor(sqrt((N * sum(x*x) - sum(x)^2) / N^2)).
 * Do NOT square a truncated mean: that loses small AC signals on a DC offset.
 * All subtraction is performed before division, in 64+LOG2_N bits.
 * Supported LOG2_N: 1..16. 64-bit sumsq is sufficient for this range.
 *
 * Input/output use ordinary valid/ready handshakes. Output is held stable
 * while out_valid && !out_ready. A request is accepted only when in_ready.
 * Dependency: the uploaded isqrt_u64.v (32-iteration integer square root).
 */
module rms_ac_calc #(
    parameter integer LOG2_N = 11
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_valid,
    output wire                       in_ready,
    input  wire signed [24+LOG2_N-1:0] sum_in,
    input  wire [63:0]                sumsq_in,
    output wire                       out_valid,
    input  wire                       out_ready,
    output reg  [23:0]                rms_ac,
    output reg                        arithmetic_error,
    output wire                       busy
);
    localparam integer SUM_W  = 24 + LOG2_N;
    localparam integer PROD_W = 2 * SUM_W;
    localparam integer NUM_W  = 64 + LOG2_N;
    localparam [2:0] IDLE=0, MULTIPLY=1, SUBTRACT=2,
                     START_SQRT=3, WAIT_SQRT=4, OUTPUT_RESULT=5;

    reg [2:0] state;
    reg signed [SUM_W-1:0] sum_hold;
    reg [PROD_W-1:0] sum_squared;
    reg [NUM_W-1:0] scaled_sumsq;
    reg [NUM_W-1:0] numerator;
    reg [63:0] sqrt_value;
    reg sqrt_start;
    wire [31:0] sqrt_result;
    wire sqrt_done;
    wire sqrt_busy_unused;
    wire signed [PROD_W-1:0] square_signed = sum_hold * sum_hold;
    wire [NUM_W-1:0] square_extended =
        {{(NUM_W-PROD_W){1'b0}}, sum_squared};
    wire [NUM_W-1:0] variance_wide = numerator >> (2*LOG2_N);

    assign in_ready = (state == IDLE);
    assign out_valid = (state == OUTPUT_RESULT);
    assign busy = (state != IDLE);

    isqrt_u64 u_sqrt (
        .clk(clk), .rst_n(rst_n), .start(sqrt_start),
        .radicand(sqrt_value), .result(sqrt_result),
        .busy(sqrt_busy_unused), .done(sqrt_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            sum_hold <= 0;
            sum_squared <= 0;
            scaled_sumsq <= 0;
            numerator <= 0;
            sqrt_value <= 0;
            sqrt_start <= 0;
            rms_ac <= 0;
            arithmetic_error <= 0;
        end else begin
            sqrt_start <= 1'b0;
            case (state)
                IDLE: if (in_valid) begin
                    sum_hold <= sum_in;
                    scaled_sumsq <= {sumsq_in, {LOG2_N{1'b0}}};
                    arithmetic_error <= 1'b0;
                    state <= MULTIPLY;
                end
                MULTIPLY: begin
                    // Assignment context retains the complete 2*SUM_W product.
                    sum_squared <= square_signed[PROD_W-1:0];
                    state <= SUBTRACT;
                end
                SUBTRACT: begin
                    if (scaled_sumsq >= square_extended) begin
                        numerator <= scaled_sumsq - square_extended;
                    end else begin
                        // Impossible for a consistent, non-overflowed window.
                        numerator <= 0;
                        arithmetic_error <= 1'b1;
                    end
                    state <= START_SQRT;
                end
                START_SQRT: begin
                    sqrt_value <= variance_wide[63:0];
                    sqrt_start <= 1'b1;
                    state <= WAIT_SQRT;
                end
                WAIT_SQRT: if (sqrt_done) begin
                    if (|sqrt_result[31:24]) begin
                        rms_ac <= 24'hFFFFFF;
                        arithmetic_error <= 1'b1;
                    end else begin
                        rms_ac <= sqrt_result[23:0];
                    end
                    state <= OUTPUT_RESULT;
                end
                OUTPUT_RESULT: if (out_ready) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end

    // synthesis translate_off
    initial begin
        if (LOG2_N < 1 || LOG2_N > 16)
            $fatal(1, "rms_ac_calc requires LOG2_N in 1..16");
    end
    // synthesis translate_on
endmodule
`default_nettype wire
