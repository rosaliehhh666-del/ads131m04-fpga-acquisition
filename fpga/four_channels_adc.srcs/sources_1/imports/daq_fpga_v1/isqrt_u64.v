`timescale 1ns / 1ps
/*
 * Unsigned 64-bit integer square root.
 * Latency: 32 clocks after start.
 * result = floor(sqrt(radicand))
 */
module isqrt_u64(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [63:0] radicand,
    output reg  [31:0] result,
    output reg         busy,
    output reg         done
);

reg [63:0] x_work;
reg [65:0] r_work;
reg [31:0] q_work;
reg [5:0]  count;

wire [65:0] r_shift =
    (r_work << 2) | {{64{1'b0}}, x_work[63:62]};

wire [31:0] q_shift = q_work << 1;
wire [32:0] trial    = {q_shift, 1'b1};

wire take = (r_shift >= {{33{1'b0}}, trial});

wire [65:0] r_next =
    take ? (r_shift - {{33{1'b0}}, trial}) : r_shift;

wire [31:0] q_next =
    take ? (q_shift | 32'd1) : q_shift;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x_work <= 64'd0;
        r_work <= 66'd0;
        q_work <= 32'd0;
        count  <= 6'd0;
        result <= 32'd0;
        busy   <= 1'b0;
        done   <= 1'b0;
    end else begin
        done <= 1'b0;

        if (start && !busy) begin
            x_work <= radicand;
            r_work <= 66'd0;
            q_work <= 32'd0;
            count  <= 6'd0;
            busy   <= 1'b1;
        end
        else if (busy) begin
            x_work <= x_work << 2;
            r_work <= r_next;
            q_work <= q_next;

            if (count == 6'd31) begin
                result <= q_next;
                busy   <= 1'b0;
                done   <= 1'b1;
            end else begin
                count <= count + 1'b1;
            end
        end
    end
end

endmodule
