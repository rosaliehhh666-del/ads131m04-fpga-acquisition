`timescale 1ns / 1ps
/*
 * 96-bit synchronous FIFO for one complete four-channel sample:
 *   {CH3[23:0], CH2[23:0], CH1[23:0], CH0[23:0]}
 *
 * XPM FIFO is used so Vivado infers block RAM reliably.
 * READ_MODE = "fwft" is important for continuous FFT AXI feeding.
 */
module sample_fifo_xpm #(
    parameter integer FIFO_DEPTH = 4096,
    parameter integer COUNT_W    = 13
)(
    input  wire          clk,
    input  wire          rst,

    input  wire [95:0]   din,
    input  wire          wr_en,
    output wire          full,
    output wire          overflow,

    output wire [95:0]   dout,
    input  wire          rd_en,
    output wire          empty,
    output wire          data_valid,
    output wire [COUNT_W-1:0] data_count,
    output wire          underflow
);

wire [COUNT_W-1:0] wr_data_count;
wire wr_rst_busy;
wire rd_rst_busy;

xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE    ("block"),
    .ECC_MODE            ("no_ecc"),

    .FIFO_WRITE_DEPTH    (FIFO_DEPTH),

    .WRITE_DATA_WIDTH    (96),
    .WR_DATA_COUNT_WIDTH (COUNT_W),

    .PROG_FULL_THRESH    (FIFO_DEPTH-16),
    .FULL_RESET_VALUE    (0),

    .READ_MODE           ("fwft"),
    .FIFO_READ_LATENCY   (0),

    .READ_DATA_WIDTH     (96),
    .RD_DATA_COUNT_WIDTH (COUNT_W),

    .PROG_EMPTY_THRESH   (16),
    .DOUT_RESET_VALUE    ("0"),

    .USE_ADV_FEATURES    ("1707"),
    .WAKEUP_TIME         (0)
) u_fifo (
    .sleep          (1'b0),
    .rst            (rst),
    .wr_clk         (clk),

    .wr_en          (wr_en),
    .din            (din),
    .full           (full),
    .overflow       (overflow),
    .wr_data_count  (wr_data_count),
    .wr_rst_busy    (wr_rst_busy),
    .almost_full    (),
    .prog_full      (),
    .wr_ack         (),

    .rd_en          (rd_en),
    .dout           (dout),
    .empty          (empty),
    .underflow      (underflow),
    .rd_data_count  (data_count),
    .rd_rst_busy    (rd_rst_busy),
    .almost_empty   (),
    .prog_empty     (),
    .data_valid     (data_valid),

    .injectsbiterr  (1'b0),
    .injectdbiterr  (1'b0),
    .sbiterr        (),
    .dbiterr        ()
);

endmodule
