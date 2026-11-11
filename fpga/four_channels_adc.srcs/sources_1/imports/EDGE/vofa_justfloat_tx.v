`timescale 1ns / 1ps
/*
 * vofa_justfloat_tx.v
 *
 * Sends the FPGA's current processed ADS131M04 results to VOFA+ using
 * the JustFloat protocol:
 *   12 x IEEE-754 float32, little endian
 *   frame tail = 00 00 80 7F
 *
 * Channel order in VOFA+:
 *   0  CH0_VPP_code
 *   1  CH0_RMS_code
 *   2  CH0_FREQ_Hz
 *   3  CH1_VPP_code
 *   4  CH1_RMS_code
 *   5  CH1_FREQ_Hz
 *   6  CH2_VPP_code
 *   7  CH2_RMS_code
 *   8  CH2_FREQ_Hz
 *   9  CH3_VPP_code
 *   10 CH3_RMS_code
 *   11 CH3_FREQ_Hz
 *
 * A frame is sent after both the time-domain metrics and FFT frequency
 * results have produced a fresh update. This keeps UART bandwidth low and
 * naturally follows the 2048-sample processing frame rate.
 *
 * NOTE:
 *   VPP/RMS are the existing FPGA ADC-code results, not volts.
 */
module vofa_justfloat_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115_200
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [24:0] vpp0,
    input  wire [24:0] vpp1,
    input  wire [24:0] vpp2,
    input  wire [24:0] vpp3,

    input  wire [23:0] rms0,
    input  wire [23:0] rms1,
    input  wire [23:0] rms2,
    input  wire [23:0] rms3,
    input  wire        metrics_valid,

    input  wire [31:0] freq_hz0,
    input  wire [31:0] freq_hz1,
    input  wire [31:0] freq_hz2,
    input  wire [31:0] freq_hz3,
    input  wire        fft_peak_valid,

    output wire        uart_tx_o,
    output wire        frame_busy
);

/* Unsigned integer -> IEEE-754 single precision float.
 * Exact for all integers up to 2^24; larger values are truncated to the
 * available 24-bit float significand, which is sufficient for visualization.
 */
function [31:0] u32_to_float;
    input [31:0] value;
    integer i;
    integer msb_pos;
    reg found;
    reg [7:0] exponent;
    reg [31:0] normalized;
    begin
        if (value == 32'd0) begin
            u32_to_float = 32'h00000000;
        end else begin
            msb_pos = 0;
            found   = 1'b0;
            for (i = 31; i >= 0; i = i - 1) begin
                if (!found && value[i]) begin
                    msb_pos = i;
                    found   = 1'b1;
                end
            end

            exponent = 8'd127 + msb_pos;

            if (msb_pos <= 23)
                normalized = value << (23 - msb_pos);
            else
                normalized = value >> (msb_pos - 23);

            u32_to_float = {1'b0, exponent, normalized[22:0]};
        end
    end
endfunction

reg metrics_seen;
reg freq_seen;
reg frame_active;
reg [5:0] byte_index;
reg [24:0] timer_cnt;

reg [31:0] f0;
reg [31:0] f1;
reg [31:0] f2;
reg [31:0] f3;
reg [31:0] f4;
reg [31:0] f5;
reg [31:0] f6;
reg [31:0] f7;
reg [31:0] f8;
reg [31:0] f9;
reg [31:0] f10;
reg [31:0] f11;

reg [31:0] selected_word;
reg [7:0] uart_data;
wire uart_ready;
wire uart_valid = frame_active;

always @* begin
    case (byte_index[5:2])
        4'd0:  selected_word = f0;
        4'd1:  selected_word = f1;
        4'd2:  selected_word = f2;
        4'd3:  selected_word = f3;
        4'd4:  selected_word = f4;
        4'd5:  selected_word = f5;
        4'd6:  selected_word = f6;
        4'd7:  selected_word = f7;
        4'd8:  selected_word = f8;
        4'd9:  selected_word = f9;
        4'd10: selected_word = f10;
        4'd11: selected_word = f11;
        default: selected_word = 32'd0;
    endcase

    if (byte_index < 6'd48) begin
        case (byte_index[1:0])
            2'd0: uart_data = selected_word[7:0];
            2'd1: uart_data = selected_word[15:8];
            2'd2: uart_data = selected_word[23:16];
            default: uart_data = selected_word[31:24];
        endcase
    end else begin
        case (byte_index)
            6'd48: uart_data = 8'h00;
            6'd49: uart_data = 8'h00;
            6'd50: uart_data = 8'h80;
            default: uart_data = 8'h7F;
        endcase
    end
end

uart_tx #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD     (BAUD)
) u_uart_tx (
    .clk      (clk),
    .rst_n    (rst_n),
    .tx_data  (uart_data),
    .tx_valid (uart_valid),
    .tx_ready (uart_ready),
    .tx       (uart_tx_o)
);

assign frame_busy = frame_active || !uart_ready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        metrics_seen <= 1'b0;
        freq_seen    <= 1'b0;
        frame_active <= 1'b0;
        byte_index   <= 6'd0;
        timer_cnt    <= 25'd0;

        f0  <= 32'd0; f1  <= 32'd0; f2  <= 32'd0;
        f3  <= 32'd0; f4  <= 32'd0; f5  <= 32'd0;
        f6  <= 32'd0; f7  <= 32'd0; f8  <= 32'd0;
        f9  <= 32'd0; f10 <= 32'd0; f11 <= 32'd0;
    end else begin
        /* Timer fallback: trigger a frame every ~335ms (2^24 cycles at 50MHz)
         * even if metrics_valid never arrives. This ensures VOFA+ always
         * gets data once the UART mux switches to the VOFA output. */
        if (timer_cnt == 25'd0) begin
            metrics_seen <= 1'b1;
            timer_cnt    <= 25'd16777215; // 2^24 - 1
        end else begin
            timer_cnt <= timer_cnt - 1'b1;
        end

        /* Start a new VOFA frame when metrics are ready.
         * freq_seen is OR'd in so frames still flow even if FFT is not running. */
        if (!frame_active && metrics_seen) begin
            f0  <= u32_to_float({7'd0, vpp0});
            f1  <= u32_to_float({8'd0, rms0});
            f2  <= u32_to_float(freq_hz0);

            f3  <= u32_to_float({7'd0, vpp1});
            f4  <= u32_to_float({8'd0, rms1});
            f5  <= u32_to_float(freq_hz1);

            f6  <= u32_to_float({7'd0, vpp2});
            f7  <= u32_to_float({8'd0, rms2});
            f8  <= u32_to_float(freq_hz2);

            f9  <= u32_to_float({7'd0, vpp3});
            f10 <= u32_to_float({8'd0, rms3});
            f11 <= u32_to_float(freq_hz3);

            byte_index   <= 6'd0;
            frame_active <= 1'b1;
            metrics_seen <= 1'b0;
            freq_seen    <= 1'b0;
            timer_cnt    <= 25'd16777215; // reset timer after frame start
        end

        /* One byte is consumed whenever the UART is idle/ready. */
        if (frame_active && uart_ready) begin
            if (byte_index == 6'd51) begin
                frame_active <= 1'b0;
                byte_index   <= 6'd0;
            end else begin
                byte_index <= byte_index + 1'b1;
            end
        end

        /* New events win over the clear above, so updates are not lost. */
        if (metrics_valid)
            metrics_seen <= 1'b1;

        if (fft_peak_valid)
            freq_seen <= 1'b1;
    end
end

endmodule
