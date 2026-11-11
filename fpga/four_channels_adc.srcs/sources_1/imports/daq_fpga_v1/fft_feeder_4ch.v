`timescale 1ns / 1ps
/*
 * Converts a 96-bit four-channel FWFT FIFO into one continuous AXI4-Stream
 * FFT frame.
 *
 * Xilinx FFT IP configuration expected:
 *   - 4 channels
 *   - fixed point
 *   - 16-bit input
 *   - real input, imaginary part driven to zero
 *   - transform length = 2^LOG2_N (2048 by default)
 *   - Burst architecture
 *
 * TDATA layout from LSB:
 *   CH0 RE, CH0 IM, CH1 RE, CH1 IM, CH2 RE, CH2 IM, CH3 RE, CH3 IM
 */
module fft_feeder_4ch #(
    parameter integer LOG2_N      = 11,
    parameter integer FIFO_COUNT_W = 13,
    parameter integer FFT_LSB_DROP = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire [95:0]           fifo_dout,
    input  wire                  fifo_empty,
    input  wire [FIFO_COUNT_W-1:0] fifo_level,
    output wire                  fifo_rd_en,

    output wire [127:0]          s_axis_data_tdata,
    output wire                  s_axis_data_tvalid,
    input  wire                  s_axis_data_tready,
    output wire                  s_axis_data_tlast,

    output reg                   frame_start,
    output reg                   frame_active
);

localparam integer N = (1 << LOG2_N);

reg [LOG2_N-1:0] sample_index;

wire signed [23:0] ch0_24 = fifo_dout[23:0];
wire signed [23:0] ch1_24 = fifo_dout[47:24];
wire signed [23:0] ch2_24 = fifo_dout[71:48];
wire signed [23:0] ch3_24 = fifo_dout[95:72];

/*
 * Keep the upper 16 bits after dropping low ADC noise bits.
 * For V1, PGA/range control should keep useful signals near ADC full-scale.
 */
wire signed [15:0] ch0_16 = ch0_24 >>> FFT_LSB_DROP;
wire signed [15:0] ch1_16 = ch1_24 >>> FFT_LSB_DROP;
wire signed [15:0] ch2_16 = ch2_24 >>> FFT_LSB_DROP;
wire signed [15:0] ch3_16 = ch3_24 >>> FFT_LSB_DROP;

assign s_axis_data_tdata = {
    16'sd0, ch3_16,
    16'sd0, ch2_16,
    16'sd0, ch1_16,
    16'sd0, ch0_16
};

assign s_axis_data_tvalid =
    frame_active && !fifo_empty;

assign s_axis_data_tlast =
    s_axis_data_tvalid &&
    (sample_index == N-1);

assign fifo_rd_en =
    s_axis_data_tvalid &&
    s_axis_data_tready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_index <= {LOG2_N{1'b0}};
        frame_start  <= 1'b0;
        frame_active <= 1'b0;
    end else begin
        frame_start <= 1'b0;

        if (!frame_active) begin
            sample_index <= {LOG2_N{1'b0}};

            if (fifo_level >= N) begin
                frame_active <= 1'b1;
                frame_start  <= 1'b1;
            end
        end
        else if (fifo_rd_en) begin
            if (sample_index == N-1) begin
                sample_index <= {LOG2_N{1'b0}};
                frame_active <= 1'b0;
            end else begin
                sample_index <= sample_index + 1'b1;
            end
        end
    end
end

endmodule
