`timescale 1ns / 1ps
/*
 * Spectrum magnitude-squared + peak-bin extraction for a four-channel
 * 16-bit fixed-point Xilinx FFT output.
 *
 * Assumes natural-order output and XK_INDEX enabled in TUSER.
 * For 2048-point FFT, XK_INDEX is 11 bits.
 *
 * Only bins 1..N/2 are searched:
 *   bin 0 is ignored (DC)
 *   bins above N/2 are the redundant half for real-valued input
 */
module fft_peak_4ch #(
    parameter integer LOG2_N = 11,
    parameter integer FS_HZ  = 16000    // fDATA single source (16 kSPS)
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire [127:0]          m_axis_data_tdata,
    input  wire                  m_axis_data_tvalid,
    input  wire [LOG2_N-1:0]     bin_index,
    input  wire                  m_axis_data_tlast,

    output reg  [LOG2_N-1:0]     peak_bin0,
    output reg  [LOG2_N-1:0]     peak_bin1,
    output reg  [LOG2_N-1:0]     peak_bin2,
    output reg  [LOG2_N-1:0]     peak_bin3,

    output reg  [32:0]           peak_mag0,
    output reg  [32:0]           peak_mag1,
    output reg  [32:0]           peak_mag2,
    output reg  [32:0]           peak_mag3,

    output reg  [31:0]           freq_hz0,
    output reg  [31:0]           freq_hz1,
    output reg  [31:0]           freq_hz2,
    output reg  [31:0]           freq_hz3,

    output reg                   peak_valid
);

localparam integer HALF_N = (1 << (LOG2_N-1));

wire signed [15:0] re0 = m_axis_data_tdata[15:0];
wire signed [15:0] im0 = m_axis_data_tdata[31:16];
wire signed [15:0] re1 = m_axis_data_tdata[47:32];
wire signed [15:0] im1 = m_axis_data_tdata[63:48];
wire signed [15:0] re2 = m_axis_data_tdata[79:64];
wire signed [15:0] im2 = m_axis_data_tdata[95:80];
wire signed [15:0] re3 = m_axis_data_tdata[111:96];
wire signed [15:0] im3 = m_axis_data_tdata[127:112];

wire signed [31:0] re0_sq_s = re0 * re0;
wire signed [31:0] im0_sq_s = im0 * im0;
wire signed [31:0] re1_sq_s = re1 * re1;
wire signed [31:0] im1_sq_s = im1 * im1;
wire signed [31:0] re2_sq_s = re2 * re2;
wire signed [31:0] im2_sq_s = im2 * im2;
wire signed [31:0] re3_sq_s = re3 * re3;
wire signed [31:0] im3_sq_s = im3 * im3;

wire [32:0] mag0 =
    {1'b0, re0_sq_s[31:0]} + {1'b0, im0_sq_s[31:0]};
wire [32:0] mag1 =
    {1'b0, re1_sq_s[31:0]} + {1'b0, im1_sq_s[31:0]};
wire [32:0] mag2 =
    {1'b0, re2_sq_s[31:0]} + {1'b0, im2_sq_s[31:0]};
wire [32:0] mag3 =
    {1'b0, re3_sq_s[31:0]} + {1'b0, im3_sq_s[31:0]};

reg [32:0] pmax0, pmax1, pmax2, pmax3;
reg [LOG2_N-1:0] pbin0, pbin1, pbin2, pbin3;

wire in_search_band =
    (bin_index != 0) &&
    (bin_index <= HALF_N);

wire [63:0] freq_calc0 = pbin0 * FS_HZ;
wire [63:0] freq_calc1 = pbin1 * FS_HZ;
wire [63:0] freq_calc2 = pbin2 * FS_HZ;
wire [63:0] freq_calc3 = pbin3 * FS_HZ;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pmax0 <= 33'd0; pmax1 <= 33'd0;
        pmax2 <= 33'd0; pmax3 <= 33'd0;
        pbin0 <= {LOG2_N{1'b0}};
        pbin1 <= {LOG2_N{1'b0}};
        pbin2 <= {LOG2_N{1'b0}};
        pbin3 <= {LOG2_N{1'b0}};

        peak_bin0 <= {LOG2_N{1'b0}};
        peak_bin1 <= {LOG2_N{1'b0}};
        peak_bin2 <= {LOG2_N{1'b0}};
        peak_bin3 <= {LOG2_N{1'b0}};

        peak_mag0 <= 33'd0; peak_mag1 <= 33'd0;
        peak_mag2 <= 33'd0; peak_mag3 <= 33'd0;

        freq_hz0 <= 32'd0; freq_hz1 <= 32'd0;
        freq_hz2 <= 32'd0; freq_hz3 <= 32'd0;

        peak_valid <= 1'b0;
    end else begin
        peak_valid <= 1'b0;

        if (m_axis_data_tvalid) begin
            if (bin_index == 0) begin
                pmax0 <= 33'd0; pmax1 <= 33'd0;
                pmax2 <= 33'd0; pmax3 <= 33'd0;
                pbin0 <= {LOG2_N{1'b0}};
                pbin1 <= {LOG2_N{1'b0}};
                pbin2 <= {LOG2_N{1'b0}};
                pbin3 <= {LOG2_N{1'b0}};
            end
            else if (in_search_band) begin
                if (mag0 > pmax0) begin pmax0 <= mag0; pbin0 <= bin_index; end
                if (mag1 > pmax1) begin pmax1 <= mag1; pbin1 <= bin_index; end
                if (mag2 > pmax2) begin pmax2 <= mag2; pbin2 <= bin_index; end
                if (mag3 > pmax3) begin pmax3 <= mag3; pbin3 <= bin_index; end
            end

            if (m_axis_data_tlast) begin
                peak_bin0 <= pbin0;
                peak_bin1 <= pbin1;
                peak_bin2 <= pbin2;
                peak_bin3 <= pbin3;

                peak_mag0 <= pmax0;
                peak_mag1 <= pmax1;
                peak_mag2 <= pmax2;
                peak_mag3 <= pmax3;

                freq_hz0 <= freq_calc0 >> LOG2_N;
                freq_hz1 <= freq_calc1 >> LOG2_N;
                freq_hz2 <= freq_calc2 >> LOG2_N;
                freq_hz3 <= freq_calc3 >> LOG2_N;

                peak_valid <= 1'b1;
            end
        end
    end
end

endmodule
