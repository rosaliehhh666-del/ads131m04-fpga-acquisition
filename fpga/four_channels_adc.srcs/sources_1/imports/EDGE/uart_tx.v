`timescale 1ns / 1ps
/*
 * uart_tx.v
 *
 * Simple 8-N-1 UART transmitter.
 * Default target: 50 MHz clock, 115200 baud.
 *
 * Handshake:
 *   tx_valid && tx_ready -> one byte accepted.
 */
module uart_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output wire       tx_ready,
    output reg        tx
);

localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD;
localparam integer CNT_W = (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT);

localparam [1:0] ST_IDLE  = 2'd0;
localparam [1:0] ST_START = 2'd1;
localparam [1:0] ST_DATA  = 2'd2;
localparam [1:0] ST_STOP  = 2'd3;

reg [1:0] state;
reg [CNT_W-1:0] clk_count;
reg [2:0] bit_index;
reg [7:0] data_latched;

assign tx_ready = (state == ST_IDLE);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        clk_count    <= {CNT_W{1'b0}};
        bit_index    <= 3'd0;
        data_latched <= 8'd0;
        tx           <= 1'b1;
    end else begin
        case (state)
            ST_IDLE: begin
                tx        <= 1'b1;
                clk_count <= {CNT_W{1'b0}};
                bit_index <= 3'd0;

                if (tx_valid) begin
                    data_latched <= tx_data;
                    tx           <= 1'b0;   // start bit
                    state        <= ST_START;
                end
            end

            ST_START: begin
                if (clk_count == CLKS_PER_BIT-1) begin
                    clk_count <= {CNT_W{1'b0}};
                    tx        <= data_latched[0];
                    bit_index <= 3'd0;
                    state     <= ST_DATA;
                end else begin
                    clk_count <= clk_count + 1'b1;
                end
            end

            ST_DATA: begin
                if (clk_count == CLKS_PER_BIT-1) begin
                    clk_count <= {CNT_W{1'b0}};

                    if (bit_index == 3'd7) begin
                        tx    <= 1'b1;       // stop bit
                        state <= ST_STOP;
                    end else begin
                        bit_index <= bit_index + 1'b1;
                        tx        <= data_latched[bit_index + 1'b1];
                    end
                end else begin
                    clk_count <= clk_count + 1'b1;
                end
            end

            ST_STOP: begin
                if (clk_count == CLKS_PER_BIT-1) begin
                    clk_count <= {CNT_W{1'b0}};
                    tx        <= 1'b1;
                    state     <= ST_IDLE;
                end else begin
                    clk_count <= clk_count + 1'b1;
                end
            end

            default: begin
                state <= ST_IDLE;
                tx    <= 1'b1;
            end
        endcase
    end
end

endmodule
