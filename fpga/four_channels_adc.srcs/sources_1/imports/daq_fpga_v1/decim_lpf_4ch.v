`timescale 1ns / 1ps
`default_nettype none

/*
 * decim_lpf_4ch.v
 *
 * P0-3: anti-alias low-pass + decimation for the ADS131M04 low-frequency path.
 * Replaces the previous direct "/8 pick one sample" raw FIFO decimation:
 *   native 16 kSPS -> FIR low-pass (257 taps, Hamming, fc=890 Hz) -> /8 -> 2 kSPS
 *
 * The native 16 kSPS path (metrics / FFT) is untouched: this module only
 * feeds the raw readout FIFO, so MCU can fetch both native-rate decimated
 * data and the diagnostics.
 *
 * Implementation (mirrors the reference DDC FIR structure, serial MAC):
 *   - 4 circular sample buffers, TAPS deep each (24-bit signed samples)
 *   - one signed MAC per cycle: 24b * 16b(Q15) product, 48-bit accumulator
 *   - channels processed serially: 4 * TAPS cycles per input sample.
 *     At 16 kSPS / 50 MHz one input period = 3125 cycles > 4*257 = 1028, OK.
 *   - decimation by DECIM: the "every DECIM-th input sample" decision is
 *     latched when the sample arrives (decim_tick is only meaningful during
 *     sample_valid), then applied when the channel filter completes.
 *   - output: 48-bit accumulator >>15 (Q15 de-scale), saturated to 24 bits.
 *
 * Coeffs: scripts/design_ads131_decim_fir.py -> fir_coeff_ads131_16k_2k.mem
 * (Q15 signed hex, one per line, DC gain = 1.0).
 *
 * NOTE for Vivado: $readmemh resolves relative to the Vivado working
 * directory. Keep fir_coeff_ads131_16k_2k.mem next to this file, or copy it
 * to the project root / pass an absolute path via COEFF_FILE if the build
 * cannot find it.
 */
module decim_lpf_4ch #(
    parameter integer TAPS       = 257,
    parameter integer DECIM      = 8,
    parameter integer DATA_W     = 24,
    parameter integer COEF_W     = 16,
    parameter integer ACC_W      = 48,
    parameter        COEFF_FILE  = "fir_coeff_ads131_16k_2k.mem"
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  sample_valid,      // native-rate input strobe
    input  wire signed [DATA_W-1:0] ch0_in,
    input  wire signed [DATA_W-1:0] ch1_in,
    input  wire signed [DATA_W-1:0] ch2_in,
    input  wire signed [DATA_W-1:0] ch3_in,
    output reg  signed [DATA_W-1:0] ch0_out,
    output reg  signed [DATA_W-1:0] ch1_out,
    output reg  signed [DATA_W-1:0] ch2_out,
    output reg  signed [DATA_W-1:0] ch3_out,
    output reg                  out_valid          // decimated-rate strobe
);

    // ------------------------------------------------------------------
    // Coefficients ROM
    // ------------------------------------------------------------------
    (* rom_style = "block" *) reg signed [COEF_W-1:0] coeff_rom [0:TAPS-1];
    initial $readmemh(COEFF_FILE, coeff_rom);

    // ------------------------------------------------------------------
    // Circular sample buffers (one per channel)
    // ------------------------------------------------------------------
    (* ram_style = "block" *) reg signed [DATA_W-1:0] buf_ch0 [0:TAPS-1];
    (* ram_style = "block" *) reg signed [DATA_W-1:0] buf_ch1 [0:TAPS-1];
    (* ram_style = "block" *) reg signed [DATA_W-1:0] buf_ch2 [0:TAPS-1];
    (* ram_style = "block" *) reg signed [DATA_W-1:0] buf_ch3 [0:TAPS-1];

    // ------------------------------------------------------------------
    // Pointers & FSM
    // ------------------------------------------------------------------
    localparam integer PTR_W = $clog2(TAPS);           // 9 bits for 257
    reg [PTR_W-1:0] wr_ptr;                            // next write slot
    reg [PTR_W-1:0] tap_idx;                           // 0..TAPS-1
    reg [1:0]       ch_idx;                            // 0..3

    localparam [1:0] S_IDLE=0, S_FILTER=1, S_EMIT=2;
    reg [1:0] state;

    reg [31:0] decim_cnt;
    wire decim_tick = (decim_cnt == DECIM-1);          // qualified by sample_valid

    reg signed [ACC_W-1:0] acc0, acc1, acc2, acc3;
    reg signed [DATA_W-1:0] emit0, emit1, emit2, emit3;
    reg emit_valid;

    wire [PTR_W-1:0] newest_ptr = (wr_ptr == 0) ? TAPS-1 : wr_ptr - 1'b1;
    wire [PTR_W-1:0] buf_addr = (newest_ptr >= tap_idx) ?
                                (newest_ptr - tap_idx) : (newest_ptr - tap_idx + TAPS);

    // Serial MAC data/coef mux for current (ch_idx, tap_idx)
    wire signed [DATA_W-1:0] cur_data =
        (ch_idx == 2'd0) ? buf_ch0[buf_addr] :
        (ch_idx == 2'd1) ? buf_ch1[buf_addr] :
        (ch_idx == 2'd2) ? buf_ch2[buf_addr] : buf_ch3[buf_addr];

    wire signed [COEF_W-1:0] cur_coef = coeff_rom[tap_idx];

    // 24b * 16b -> 40b signed product
    wire signed [DATA_W+COEF_W-1:0] product = cur_data * cur_coef;

    // Q15 de-scale (>>15) + saturate to DATA_W
    function [DATA_W-1:0] sat24;
        input signed [ACC_W-1:0] x;
        reg signed [ACC_W-1:0] shifted;
        begin
            shifted = x >>> 15;
            if (shifted > ((1 << (DATA_W-1)) - 1))
                sat24 = (1 << (DATA_W-1)) - 1;
            else if (shifted < -(1 << (DATA_W-1)))
                sat24 = 1 << (DATA_W-1);
            else
                sat24 = shifted[DATA_W-1:0];
        end
    endfunction

    // synthesis translate_off
    initial begin
        if (DECIM < 1)
            $fatal(1, "decim_lpf_4ch: DECIM must be >= 1");
        if (TAPS < 2 || TAPS > (1 << PTR_W))
            $fatal(1, "decim_lpf_4ch: TAPS out of range");
    end
    // synthesis translate_on

    // ------------------------------------------------------------------
    // Decimation counter (counts input samples)
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            decim_cnt <= 32'd0;
        else if (sample_valid)
            decim_cnt <= decim_tick ? 32'd0 : decim_cnt + 32'd1;
    end

    // ------------------------------------------------------------------
    // Write pointer (native rate, advances on every accepted sample)
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_ptr <= {PTR_W{1'b0}};
        else if (sample_valid)
            wr_ptr <= (wr_ptr == TAPS-1) ? {PTR_W{1'b0}} : wr_ptr + 1'b1;
    end

    // ------------------------------------------------------------------
    // Sample writes (native rate; FSM is guaranteed idle here, see budget
    // note above: 1028 filter cycles << 3125 sample period)
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (sample_valid) begin
            buf_ch0[wr_ptr] <= ch0_in;
            buf_ch1[wr_ptr] <= ch1_in;
            buf_ch2[wr_ptr] <= ch2_in;
            buf_ch3[wr_ptr] <= ch3_in;
        end
    end

    // ------------------------------------------------------------------
    // Control FSM: IDLE -> (sample) FILTER(4ch*TAPS) -> EMIT -> IDLE
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            tap_idx    <= {PTR_W{1'b0}};
            ch_idx     <= 2'd0;
            acc0       <= {ACC_W{1'b0}};
            acc1       <= {ACC_W{1'b0}};
            acc2       <= {ACC_W{1'b0}};
            acc3       <= {ACC_W{1'b0}};
            emit_valid <= 1'b0;
            emit0      <= {DATA_W{1'b0}};
            emit1      <= {DATA_W{1'b0}};
            emit2      <= {DATA_W{1'b0}};
            emit3      <= {DATA_W{1'b0}};
            ch0_out    <= {DATA_W{1'b0}};
            ch1_out    <= {DATA_W{1'b0}};
            ch2_out    <= {DATA_W{1'b0}};
            ch3_out    <= {DATA_W{1'b0}};
            out_valid  <= 1'b0;
        end else begin
            out_valid  <= 1'b0;
            emit_valid <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (sample_valid && decim_tick) begin
                        // Latched this sample's decimation decision; samples
                        // are already written this cycle; start filtering.
                        tap_idx       <= {PTR_W{1'b0}};
                        ch_idx        <= 2'd0;
                        acc0          <= {ACC_W{1'b0}};
                        acc1          <= {ACC_W{1'b0}};
                        acc2          <= {ACC_W{1'b0}};
                        acc3          <= {ACC_W{1'b0}};
                        state         <= S_FILTER;
                    end
                end
                S_FILTER: begin
                    case (ch_idx)
                        2'd0: acc0 <= acc0 + product;
                        2'd1: acc1 <= acc1 + product;
                        2'd2: acc2 <= acc2 + product;
                        2'd3: acc3 <= acc3 + product;
                    endcase

                    if (tap_idx == TAPS-1) begin
                        // Channel finished: final MAC folded into emit value
                        // (accX holds taps 0..TAPS-2; +product adds tap TAPS-1).
                        case (ch_idx)
                            2'd0: emit0 <= sat24(acc0 + product);
                            2'd1: emit1 <= sat24(acc1 + product);
                            2'd2: emit2 <= sat24(acc2 + product);
                            2'd3: emit3 <= sat24(acc3 + product);
                        endcase
                        if (ch_idx == 2'd3) begin
                            state       <= S_EMIT;
                            emit_valid  <= 1'b1;
                        end else begin
                            ch_idx  <= ch_idx + 2'd1;
                            tap_idx <= {PTR_W{1'b0}};
                        end
                    end else begin
                        tap_idx <= tap_idx + 1'b1;
                    end
                end
                S_EMIT: begin
                    state <= S_IDLE;
                    if (emit_valid) begin
                        ch0_out   <= emit0;
                        ch1_out   <= emit1;
                        ch2_out   <= emit2;
                        ch3_out   <= emit3;
                        out_valid <= 1'b1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
