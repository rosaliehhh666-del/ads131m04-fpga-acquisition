`timescale 1ns / 1ps
/*
 * daq_board_top.v  —  UART MUX: debug heartbeat + VOFA+
 *
 *   daq_core_top    — full ADC/SPI/FFT/metrics chain
 *   debug_state_uart — init NOT done: "FPGA UART OK\r\n" every 250 ms
 *                      init done:     pass through VOFA+ JustFloat
 *
 * UART output works even when ADC init fails — the heartbeat
 * is completely independent of ADC / SPI / DRDY / FFT.
 *
 * FPGA target: XC7A35T-2FGG484, 50 MHz system clock.
 */
module daq_board_top(
    input  wire clk_50m,
    input  wire rst_n,

    /* ADS131M04 */
    input  wire adc_drdy,
    input  wire adc_dout,
    output wire adc_reset_n,
    output wire adc_cs_n,
    output wire adc_sclk,
    output wire adc_din,

    /* MCU SPI: MCU master, FPGA slave; mode 0, <= 2 MHz recommended */
    input  wire mcu_cs_n,
    input  wire mcu_sclk,
    input  wire mcu_mosi,
    output wire mcu_miso,

    /* On-board USB-UART CH340E: FPGA TX -> CH340 RXD -> PC */
    output wire uart_tx,
    output wire linux_tx,
    input wire linux_rx
);

(* mark_debug = "true" *) wire        adc_init_done_int;
(* mark_debug = "true" *) wire        adc_init_error_int;
(* mark_debug = "true" *) wire [3:0]  adc_retry_cnt_int;
(* mark_debug = "true" *) wire [3:0]  adc_init_state_int;
(* mark_debug = "true" *) wire [23:0] adc_wreg_ack_int;
(* mark_debug = "true" *) wire [23:0] adc_id_word_int;

// SPI debug signals (for UART diagnostic output)
(* mark_debug = "true" *) wire [23:0] spi_tx_word0_int;
(* mark_debug = "true" *) wire [23:0] spi_rx_word0_int;
(* mark_debug = "true" *) wire [23:0] spi_rx_word1_int;
(* mark_debug = "true" *) wire        spi_busy_int;
(* mark_debug = "true" *) wire        spi_cmd_done_int;

// ============================================================
// ILA probes: ADS131M04 physical SPI signals
// ============================================================
(* mark_debug = "true" *) wire        dbg_adc_reset_n;
(* mark_debug = "true" *) wire        dbg_adc_drdy;
(* mark_debug = "true" *) wire        dbg_adc_cs_n;
(* mark_debug = "true" *) wire        dbg_adc_sclk;
(* mark_debug = "true" *) wire        dbg_adc_din;
(* mark_debug = "true" *) wire        dbg_adc_dout;

assign dbg_adc_reset_n = adc_reset_n;
IBUF u_adc_drdy_ibuf (.I(adc_drdy), .O(dbg_adc_drdy));
assign dbg_adc_cs_n    = adc_cs_n;
assign dbg_adc_sclk    = adc_sclk;
assign dbg_adc_din     = adc_din;
assign dbg_adc_dout    = adc_dout;

(* mark_debug = "true" *) wire [24:0] vpp0_int;
(* mark_debug = "true" *) wire [24:0] vpp1_int;
(* mark_debug = "true" *) wire [24:0] vpp2_int;
(* mark_debug = "true" *) wire [24:0] vpp3_int;

(* mark_debug = "true" *) wire [23:0] rms0_int;
(* mark_debug = "true" *) wire [23:0] rms1_int;
(* mark_debug = "true" *) wire [23:0] rms2_int;
(* mark_debug = "true" *) wire [23:0] rms3_int;

(* mark_debug = "true" *) wire        metrics_valid_int;

(* mark_debug = "true" *) wire [31:0] freq_hz0_int;
(* mark_debug = "true" *) wire [31:0] freq_hz1_int;
(* mark_debug = "true" *) wire [31:0] freq_hz2_int;
(* mark_debug = "true" *) wire [31:0] freq_hz3_int;

(* mark_debug = "true" *) wire        fft_peak_valid_int;
(* mark_debug = "true" *) wire        vofa_frame_busy_int;
(* mark_debug = "true" *) wire        mcu_irq_int;

wire mcu_miso_data_int;
wire mcu_miso_oe_int;

/* mcu_spi_slave_rw exports data + OE; release shared MISO when CS is high. */
assign mcu_miso = mcu_miso_oe_int ? mcu_miso_data_int : 1'bz;

/* ========================================================
 * Existing ADC acquisition / metrics / FFT core
 * ======================================================== */
localparam integer ADC_FS_HZ = 16000;
wire [95:0] screen_sample;
wire screen_valid;
wire screen_enabled;
ads_screen_control u_screen_control (.clk(clk_50m), .rst_n(rst_n),
    .rx(linux_rx), .enabled(screen_enabled));
ads_screen_tx #(.FS_HZ(ADC_FS_HZ/8)) u_screen (
    .clk(clk_50m), .rst_n(rst_n && adc_init_done_int),
    .sample_valid(screen_valid), .enabled(screen_enabled), .sample_data(screen_sample), .tx(linux_tx)
);

daq_core_top #(.ADC_FS_HZ(ADC_FS_HZ)) u_daq_core (
    .screen_sample(screen_sample),
    .screen_valid(screen_valid),
    .clk_50m         (clk_50m),
    .rst_n           (rst_n),

    .adc_drdy        (dbg_adc_drdy),
    .adc_dout        (adc_dout),
    .adc_reset_n     (adc_reset_n),
    .adc_cs_n        (adc_cs_n),
    .adc_sclk        (adc_sclk),
    .adc_din         (adc_din),

    .adc_init_done   (adc_init_done_int),
    .adc_init_error  (adc_init_error_int),
    .adc_retry_count (adc_retry_cnt_int),
    .adc_init_state  (adc_init_state_int),
    .adc_wreg_ack    (adc_wreg_ack_int),
    .adc_id_word     (adc_id_word_int),
    .spi_tx_word0    (spi_tx_word0_int),
    .spi_rx_word0    (spi_rx_word0_int),
    .spi_rx_word1    (spi_rx_word1_int),
    .spi_busy_out    (spi_busy_int),
    .spi_cmd_done_out(spi_cmd_done_int),

    .vpp0            (vpp0_int),
    .vpp1            (vpp1_int),
    .vpp2            (vpp2_int),
    .vpp3            (vpp3_int),

    .rms0            (rms0_int),
    .rms1            (rms1_int),
    .rms2            (rms2_int),
    .rms3            (rms3_int),

    .metrics_valid   (metrics_valid_int),

    .freq_hz0        (freq_hz0_int),
    .freq_hz1        (freq_hz1_int),
    .freq_hz2        (freq_hz2_int),
    .freq_hz3        (freq_hz3_int),

    .fft_peak_valid  (fft_peak_valid_int),

    .mcu_cs_n        (mcu_cs_n),
    .mcu_sclk        (mcu_sclk),
    .mcu_mosi        (mcu_mosi),
    .mcu_miso        (mcu_miso_data_int),
    .mcu_miso_oe     (mcu_miso_oe_int),
    .mcu_irq         (mcu_irq_int)
);

/* VOFA+ module is kept for later but its output is NOT connected to uart_tx
   during this UART isolation test phase. */
wire vofa_uart_tx_int;

vofa_justfloat_tx #(
    .CLK_FREQ (50_000_000),
    .BAUD     (115_200)
) u_vofa_uart (
    .clk             (clk_50m),
    .rst_n           (rst_n),

    .vpp0            (vpp0_int),
    .vpp1            (vpp1_int),
    .vpp2            (vpp2_int),
    .vpp3            (vpp3_int),

    .rms0            (rms0_int),
    .rms1            (rms1_int),
    .rms2            (rms2_int),
    .rms3            (rms3_int),
    .metrics_valid   (metrics_valid_int),

    .freq_hz0        (freq_hz0_int),
    .freq_hz1        (freq_hz1_int),
    .freq_hz2        (freq_hz2_int),
    .freq_hz3        (freq_hz3_int),
    .fft_peak_valid  (fft_peak_valid_int),

    .uart_tx_o       (vofa_uart_tx_int),
    .frame_busy      (vofa_frame_busy_int)
);

/* ========================================================
 * UART MUX: debug heartbeat when init not done,
 *           VOFA+ JustFloat when init done.
 *
 * init NOT done: "FPGA UART OK\r\n" every 250 ms (ASCII)
 * init done:     pass through VOFA+ JustFloat data
 * ======================================================== */
debug_state_uart #(
    .CLK_FREQ (50_000_000),
    .BAUD     (115_200)
) u_debug_uart (
    .clk           (clk_50m),
    .rst_n         (rst_n),

    .init_state    (adc_init_state_int),
    .retry_cnt     (adc_retry_cnt_int),
    .id_word       (adc_id_word_int),

    .spi_tx_word0  (spi_tx_word0_int),
    .spi_rx_word0  (spi_rx_word0_int),
    .spi_rx_word1  (spi_rx_word1_int),

    .spi_busy      (spi_busy_int),
    .spi_cmd_done  (spi_cmd_done_int),

    .init_done     (adc_init_done_int),

    .vofa_uart_tx  (vofa_uart_tx_int),

    .uart_tx       (uart_tx)
);

endmodule
