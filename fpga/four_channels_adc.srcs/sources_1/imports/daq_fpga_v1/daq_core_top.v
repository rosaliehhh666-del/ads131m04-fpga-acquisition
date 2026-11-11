`timescale 1ns / 1ps
`default_nettype none

`ifndef DAQ_FFT_MODULE
`define DAQ_FFT_MODULE xfft_four_channles
`endif

/*
 * daq_core_top.v
 *
 * V1 FPGA core integration
 *
 * ADS131M04
 *   -> 4 x signed 24-bit synchronous samples
 *   -> Vpp / RMS / Mean
 *   -> 96-bit XPM FIFO
 *   -> 4-channel Xilinx FFT IP
 *   -> Magnitude^2 / Peak Bin / Frequency
 *
 * Target:
 *   FPGA       : XC7A35T
 *   System CLK : 50 MHz
 *   ADC        : ADS131M04
 *   Fs         : 16 kSPS / channel
 *   FFT        : 2048 points / 4 channels / fixed-point 16-bit
 *
 * ADC initialization:
 *
 *   power-up delay
 *       |
 *       v
 *   wait DRDY
 *       |
 *       v
 *   WREG CLOCK = 0x0F06
 *       |
 *       v
 *   wait next DRDY
 *       |
 *       v
 *   NULL frame
 *       |
 *       v
 *   check 0x4180xx response
 *       |
 *       v
 *   enable normal streaming
 */

module daq_core_top #(
    parameter integer ADC_GAIN1 = 16'h0000,   // CH1/CH2 PGA (GAIN1 reg value)
    parameter integer ADC_GAIN2 = 16'h0000,   // CH3/CH4 PGA (GAIN2 reg value)
    parameter integer ADC_CFG   = 16'h0000,   // CFG reg (CRC_EN[3:0] etc.)
    parameter integer ADC_FS_HZ = 16000       // configured fDATA, single source
)(
    input  wire         clk_50m,
    input  wire         rst_n,

    // =========================================================
    // ADS131M04 physical interface
    // =========================================================

    input  wire         adc_drdy,
    input  wire         adc_dout,
    output wire         adc_reset_n,
    output wire         adc_cs_n,
    output wire         adc_sclk,
    output wire         adc_din,

    output wire [95:0]  screen_sample,
    output wire         screen_valid,

    // ADC initialization status
    output wire         adc_init_done,
    output wire         adc_init_error,
    output wire [3:0]   adc_retry_count,
    output wire [3:0]   adc_init_state,
    output wire [23:0]  adc_wreg_ack,
    output wire [23:0]  adc_id_word,
    output wire [23:0]  spi_tx_word0,
    output wire [23:0]  spi_rx_word0,
    output wire [23:0]  spi_rx_word1,
    output wire         spi_busy_out,
    output wire         spi_cmd_done_out,

    // =========================================================
    // Time-domain results
    // =========================================================

    output wire [24:0]  vpp0,
    output wire [24:0]  vpp1,
    output wire [24:0]  vpp2,
    output wire [24:0]  vpp3,

    output wire [23:0]  rms0,
    output wire [23:0]  rms1,
    output wire [23:0]  rms2,
    output wire [23:0]  rms3,

    output wire         metrics_valid,

    // =========================================================
    // FFT results
    // =========================================================

    output wire [31:0]  freq_hz0,
    output wire [31:0]  freq_hz1,
    output wire [31:0]  freq_hz2,
    output wire [31:0]  freq_hz3,

    output wire         fft_peak_valid,

    // =========================================================
    // MCU SPI
    // =========================================================

    input  wire         mcu_cs_n,
    input  wire         mcu_sclk,
    input  wire         mcu_mosi,
    output wire         mcu_miso,
    output wire         mcu_miso_oe,
    output wire         mcu_irq
);


    // =========================================================
    // Internal reset
    //
    // Asynchronous assertion,
    // synchronous release.
    // =========================================================

    (* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync;

    always @(posedge clk_50m or negedge rst_n) begin

        if (!rst_n)
            reset_sync <= 3'b000;
        else
            reset_sync <= {
                reset_sync[1:0],
                1'b1
            };

    end

    wire core_rst_n = reset_sync[2];


    // =========================================================
    // XPM FIFO reset
    //
    // XPM reset is synchronous to clk_50m.
    // =========================================================

    reg fifo_reset;

    always @(posedge clk_50m) begin

        fifo_reset <= !core_rst_n;

    end


    // =========================================================
    // Global fault signals
    // =========================================================

    wire [31:0] error_events;

    reg fft_path_fault;

    wire fft_sample_drop;
    wire fft_protocol_error;


    // =========================================================
    // ADS131M04 interface
    //
    // IMPORTANT:
    //
    // There is ONLY ONE signal set here.
    //
    // ads131m04_init and ads131m04_spi share exactly
    // the same command/data wires.
    // =========================================================

    wire         spi_busy;

    wire         cmd_start;
    wire [143:0] cmd_tx_frame;

    wire [143:0] cmd_rx_frame;
    wire         cmd_done;

    wire         stream_enable;

    wire signed [23:0] ch0;
    wire signed [23:0] ch1;
    wire signed [23:0] ch2;
    wire signed [23:0] ch3;

    wire         sample_valid;

    wire [23:0]  status_word;
    wire [23:0]  crc_word;

    wire         spi_overrun;

    // One-clock synchronized DRDY falling-edge pulse.
    wire         adc_drdy_event;


    // =========================================================
    // ADS131M04 initialization
    // =========================================================

    ads131m04_init #(
        .CLK_FREQ(50_000_000),
        .STARTUP_DELAY_CYCLES(1_000_000),
        .FS_HZ(ADC_FS_HZ),
        .GAIN1_VALUE(ADC_GAIN1),
        .GAIN2_VALUE(ADC_GAIN2),
        .CFG_VALUE(ADC_CFG)
    ) u_adc_init (
        .clk           (clk_50m),
        .rst_n         (core_rst_n),

        .spi_busy      (spi_busy),
        .cmd_done      (cmd_done),
        .cmd_rx_frame  (cmd_rx_frame),

        // Initialization is synchronized to DRDY.
        .drdy_event    (adc_drdy_event),

        .cfg_reconfig  (cfg_reconfig),

        .cmd_start     (cmd_start),
        .cmd_tx_frame  (cmd_tx_frame),

        .stream_enable (stream_enable),
        .adc_reset_n   (adc_reset_n),

        .init_done     (adc_init_done),
        .init_error    (adc_init_error),

        .retry_count   (adc_retry_count),
        .init_state    (adc_init_state),
        .wreg_ack_word (adc_wreg_ack),
        .id_word       (adc_id_word),
        .debug_tx_word0(init_debug_tx_word0),

        .cfg_id        (cfg_id),
        .fs_hz         (fs_hz_actual),
        .gain1_readback(gain1_readback),
        .gain2_readback(gain2_readback),
        .cfg_readback  (cfg_readback)
    );

    wire [23:0] init_debug_tx_word0;
    wire        cfg_reconfig; 
    assign cfg_reconfig = 1'b0;
    wire [31:0] cfg_id;
    wire [31:0] fs_hz_actual;
    wire [15:0] gain1_readback;
    wire [15:0] gain2_readback;
    wire [15:0] cfg_readback;

    wire [23:0] debug_tx_word0_spi;

    // SPI debug outputs for UART diagnostic
    // debug_tx_word0 is latched at cmd_start by the SPI master,
    // so it shows the actual command that was sent (not the next one).
    assign spi_tx_word0     = debug_tx_word0_spi;
    assign spi_rx_word0     = cmd_rx_frame[143:120];
    assign spi_rx_word1     = cmd_rx_frame[119:96];
    assign spi_busy_out     = spi_busy;
    assign spi_cmd_done_out = cmd_done;


    // =========================================================
    // ADS131M04 SPI
    // =========================================================

    ads131m04_spi #(
        .CLK_FREQ (50_000_000),
        .CMD_SCLK_FREQ   (500_000),
        .STREAM_SCLK_FREQ(4_000_000)
    ) u_adc_spi (
        .clk(clk_50m),
        .rst_n(core_rst_n),

        // Physical ADC
        .drdy(adc_drdy),
        .dout(adc_dout),

        .cs_n(adc_cs_n),
        .sclk(adc_sclk),
        .din(adc_din),

        // Normal stream enable
        .stream_enable(stream_enable),

        // Command interface
        .cmd_start(cmd_start),
        .cmd_tx_frame(cmd_tx_frame),

        .cmd_rx_frame(cmd_rx_frame),
        .cmd_done(cmd_done),

        // ADC data
        .ch0_data(ch0),
        .ch1_data(ch1),
        .ch2_data(ch2),
        .ch3_data(ch3),

        .status_word(status_word),
        .crc_word(crc_word),

        .sample_valid(sample_valid),

        // SPI status
        .busy(spi_busy),
        .overrun(spi_overrun),

        // DRDY event
        .drdy_event(adc_drdy_event),

        // Debug
        .debug_tx_word0(debug_tx_word0_spi)
    );


    // =========================================================
    // P0-4 / P0-5: acquisition diagnostics (kept OUT of the SPI
    // driver on purpose — the driver is left untouched).
    //
    //   - CRC-8 verification of stream frames, computed in parallel
    //     over the decoded 120-bit payload {status, ch0..ch3}.
    //   - DRDY event count / received frame count / accumulated
    //     overrun / accepted-frame sequence, all counted here.
    // =========================================================

    wire [7:0]  crc_computed;
    wire        crc_err;
    wire        cfg_crc_en = |ADC_CFG[3:0];   // CRC_EN[3:0] in CFG register

    crc8_check_120 u_crc_check (
        .clk         (clk_50m),
        .rst_n       (core_rst_n),
        .check_en    (cfg_crc_en),
        .status_word (status_word),
        .ch0         (ch0),
        .ch1         (ch1),
        .ch2         (ch2),
        .ch3         (ch3),
        .crc_word_lsb(crc_word[7:0]),
        .frame_valid (sample_valid),
        .computed_crc(crc_computed),
        .crc_err     (crc_err)
    );

    reg [31:0] drdy_event_count;
    reg [31:0] rx_frame_count;
    reg [31:0] overrun_count;
    reg [31:0] sample_seq;
    reg [31:0] crc_err_count;

    wire clear_stream_counters;   // from status_register (0x59 write pulse)

    always @(posedge clk_50m or negedge core_rst_n) begin
        if (!core_rst_n) begin
            drdy_event_count <= 32'd0;
            rx_frame_count   <= 32'd0;
            overrun_count    <= 32'd0;
            sample_seq       <= 32'd0;
            crc_err_count    <= 32'd0;
        end else if (clear_stream_counters) begin
            drdy_event_count <= 32'd0;
            rx_frame_count   <= 32'd0;
            overrun_count    <= 32'd0;
            sample_seq       <= 32'd0;
            crc_err_count    <= 32'd0;
        end else begin
            if (adc_drdy_event)
                drdy_event_count <= drdy_event_count + 32'd1;
            if (sample_valid) begin
                rx_frame_count <= rx_frame_count + 32'd1;
                sample_seq     <= sample_seq + 32'd1;
            end
            if (spi_overrun && adc_init_done)
                overrun_count <= overrun_count + 32'd1;
            if (crc_err)
                crc_err_count <= crc_err_count + 32'd1;
        end
    end


    // =========================================================
    // Time-domain metrics
    //
    // 2048-sample window
    //
    // Mean
    // Vpp
    // AC RMS
    // =========================================================

    wire signed [23:0] mean0;
    wire signed [23:0] mean1;
    wire signed [23:0] mean2;
    wire signed [23:0] mean3;

    wire [3:0] metric_lane_valid;
    wire [3:0] metric_lane_drop;
    wire [3:0] metric_lane_error;

    assign metrics_valid =
        &metric_lane_valid;


    channel_metrics_fix #(
        .LOG2_N(11)
    ) u_metrics_ch0 (
        .clk(clk_50m),
        .rst_n(core_rst_n),

        .sample_valid(sample_valid),
        .sample(ch0),

        .mean(mean0),
        .vpp(vpp0),
        .rms_ac(rms0),

        .metrics_valid(metric_lane_valid[0]),
        .metrics_ready(1'b1),

        .window_id(),
        .window_ready(),
        .calc_busy(),

        .window_dropped(metric_lane_drop[0]),
        .arithmetic_error(metric_lane_error[0])
    );


    channel_metrics_fix #(
        .LOG2_N(11)
    ) u_metrics_ch1 (
        .clk(clk_50m),
        .rst_n(core_rst_n),

        .sample_valid(sample_valid),
        .sample(ch1),

        .mean(mean1),
        .vpp(vpp1),
        .rms_ac(rms1),

        .metrics_valid(metric_lane_valid[1]),
        .metrics_ready(1'b1),

        .window_id(),
        .window_ready(),
        .calc_busy(),

        .window_dropped(metric_lane_drop[1]),
        .arithmetic_error(metric_lane_error[1])
    );


    channel_metrics_fix #(
        .LOG2_N(11)
    ) u_metrics_ch2 (
        .clk(clk_50m),
        .rst_n(core_rst_n),

        .sample_valid(sample_valid),
        .sample(ch2),

        .mean(mean2),
        .vpp(vpp2),
        .rms_ac(rms2),

        .metrics_valid(metric_lane_valid[2]),
        .metrics_ready(1'b1),

        .window_id(),
        .window_ready(),
        .calc_busy(),

        .window_dropped(metric_lane_drop[2]),
        .arithmetic_error(metric_lane_error[2])
    );


    channel_metrics_fix #(
        .LOG2_N(11)
    ) u_metrics_ch3 (
        .clk(clk_50m),
        .rst_n(core_rst_n),

        .sample_valid(sample_valid),
        .sample(ch3),

        .mean(mean3),
        .vpp(vpp3),
        .rms_ac(rms3),

        .metrics_valid(metric_lane_valid[3]),
        .metrics_ready(1'b1),

        .window_id(),
        .window_ready(),
        .calc_busy(),

        .window_dropped(metric_lane_drop[3]),
        .arithmetic_error(metric_lane_error[3])
    );


    // =========================================================
    // FFT sample FIFO
    //
    // One FIFO word:
    //
    //   {CH3, CH2, CH1, CH0}
    //
    // Width:
    //   4 x 24 = 96 bit
    //
    // Depth:
    //   4096
    // =========================================================

    wire [95:0] fifo_din;
    wire [95:0] fifo_dout;

    wire fifo_full;
    wire fifo_empty;

    wire fifo_overflow;
    wire fifo_underflow;

    wire fifo_data_valid;

    wire [12:0] fifo_level;

    wire fifo_rd_en;


    assign fifo_din = {
        ch3,
        ch2,
        ch1,
        ch0
    };


    sample_fifo_xpm #(
        .FIFO_DEPTH(4096),
        .COUNT_W   (13)
    ) u_fft_fifo (
        .clk        (clk_50m),
        .rst        (fifo_reset),

        .din        (fifo_din),

        .wr_en(
            sample_valid &&
            !fifo_full &&
            !fft_path_fault
        ),

        .full       (fifo_full),
        .overflow   (fifo_overflow),

        .dout       (fifo_dout),

        .rd_en      (fifo_rd_en),

        .empty      (fifo_empty),
        .data_valid (fifo_data_valid),

        .data_count (fifo_level),

        .underflow  (fifo_underflow)
    );


    // =========================================================
    // FFT feeder
    // =========================================================

    wire [127:0] fft_s_tdata;

    wire fft_s_tvalid;
    wire fft_s_tready;
    wire fft_s_tlast;

    wire fft_ip_s_tready;


    fft_feeder_4ch #(
        .LOG2_N       (11),
        .FIFO_COUNT_W (13),
        .FFT_LSB_DROP (8)
    ) u_fft_feeder (
        .clk(clk_50m),
        .rst_n(core_rst_n),

        .fifo_dout(fifo_dout),
        .fifo_empty(fifo_empty),
        .fifo_level(fifo_level),
        .fifo_rd_en(fifo_rd_en),

        .s_axis_data_tdata(fft_s_tdata),
        .s_axis_data_tvalid(fft_s_tvalid),
        .s_axis_data_tready(fft_s_tready),
        .s_axis_data_tlast(fft_s_tlast),

        .frame_start(),
        .frame_active()
    );


    // =========================================================
    // FFT IP signals
    // =========================================================

    wire [127:0] fft_m_tdata;
    wire [15:0]  fft_m_tuser;

    wire fft_m_tvalid;
    wire fft_m_tlast;

    wire fft_cfg_ready;

    wire fft_tlast_unexpected;
    wire fft_tlast_missing;

    wire raw_fft_peak_valid;

    reg fft_cfg_valid;
    reg fft_cfg_sent;


    // =========================================================
    // FFT configuration
    //
    // 2048 points:
    //
    // log2(N) = 11
    //
    // Forward FFT
    //
    // Four channels
    // =========================================================

    localparam [21:0] FFT_SCALE_SCH =
        22'h155555;


    localparam [103:0] FFT_CONFIG_WORD = {
        4'b0000,

        FFT_SCALE_SCH,
        FFT_SCALE_SCH,
        FFT_SCALE_SCH,
        FFT_SCALE_SCH,

        4'b1111,

        3'b000,

        5'd11
    };


    // =========================================================
    // FFT configuration handshake
    // =========================================================

    always @(posedge clk_50m or negedge core_rst_n) begin

        if (!core_rst_n) begin

            fft_cfg_valid <= 1'b0;
            fft_cfg_sent  <= 1'b0;

        end
        else begin

            if (!fft_cfg_sent) begin

                fft_cfg_valid <= 1'b1;

                if (fft_cfg_valid && fft_cfg_ready) begin

                    fft_cfg_valid <= 1'b0;
                    fft_cfg_sent  <= 1'b1;

                end

            end
            else begin

                fft_cfg_valid <= 1'b0;

            end

        end

    end


    // =========================================================
    // Do not allow FIFO consumption before FFT configuration
    // has completed.
    // =========================================================

    assign fft_s_tready =
        fft_cfg_sent ?
        fft_ip_s_tready :
        1'b0;


    // =========================================================
    // Xilinx FFT IP
    // =========================================================

    `DAQ_FFT_MODULE u_xfft (

    `ifdef DAQ_FFT_HAS_ARESETN

        .aresetn(
            core_rst_n
        ),

    `endif

        .aclk(
            clk_50m
        ),

        // CONFIG
        .s_axis_config_tdata(
            FFT_CONFIG_WORD
        ),

        .s_axis_config_tvalid(
            fft_cfg_valid
        ),

        .s_axis_config_tready(
            fft_cfg_ready
        ),

        // INPUT DATA
        .s_axis_data_tdata(
            fft_s_tdata
        ),

        .s_axis_data_tvalid(
            fft_s_tvalid &&
            fft_cfg_sent
        ),

        .s_axis_data_tready(
            fft_ip_s_tready
        ),

        .s_axis_data_tlast(
            fft_s_tlast
        ),

        // OUTPUT DATA
        .m_axis_data_tdata(
            fft_m_tdata
        ),

        .m_axis_data_tuser(
            fft_m_tuser
        ),

        .m_axis_data_tvalid(
            fft_m_tvalid
        ),

        .m_axis_data_tready(
            1'b1
        ),

        .m_axis_data_tlast(
            fft_m_tlast
        ),

        // EVENTS
        .event_frame_started(),

        .event_tlast_unexpected(
            fft_tlast_unexpected
        ),

        .event_tlast_missing(
            fft_tlast_missing
        ),

        .event_status_channel_halt(),

        .event_data_in_channel_halt(),

        .event_data_out_channel_halt()

    );


    // =========================================================
    // FFT peak search
    // =========================================================

    wire [10:0] peak_bin0;
    wire [10:0] peak_bin1;
    wire [10:0] peak_bin2;
    wire [10:0] peak_bin3;

    wire [32:0] peak_mag0;
    wire [32:0] peak_mag1;
    wire [32:0] peak_mag2;
    wire [32:0] peak_mag3;


    fft_peak_4ch #(
        .LOG2_N(11),
        .FS_HZ (ADC_FS_HZ)
    ) u_fft_peak (
        .clk(
            clk_50m
        ),

        .rst_n(
            core_rst_n
        ),

        .m_axis_data_tdata(
            fft_m_tdata
        ),

        .m_axis_data_tvalid(
            fft_m_tvalid
        ),

        .bin_index(
            fft_m_tuser[10:0]
        ),

        .m_axis_data_tlast(
            fft_m_tlast
        ),

        .peak_bin0(
            peak_bin0
        ),

        .peak_bin1(
            peak_bin1
        ),

        .peak_bin2(
            peak_bin2
        ),

        .peak_bin3(
            peak_bin3
        ),

        .peak_mag0(
            peak_mag0
        ),

        .peak_mag1(
            peak_mag1
        ),

        .peak_mag2(
            peak_mag2
        ),

        .peak_mag3(
            peak_mag3
        ),

        .freq_hz0(
            freq_hz0
        ),

        .freq_hz1(
            freq_hz1
        ),

        .freq_hz2(
            freq_hz2
        ),

        .freq_hz3(
            freq_hz3
        ),

        .peak_valid(
            raw_fft_peak_valid
        )
    );


    // =========================================================
    // Sample counter and FFT fault latch
    // =========================================================

    reg [31:0] sample_count;

    always @(posedge clk_50m or negedge core_rst_n) begin

        if (!core_rst_n) begin

            sample_count <= 32'd0;

            fft_path_fault <= 1'b0;

        end
        else begin

            if (sample_valid)
                sample_count <=
                    sample_count + 32'd1;

            if (
                fft_sample_drop ||
                fifo_underflow ||
                fft_protocol_error
            )
                fft_path_fault <= 1'b1;

        end

    end


    assign fft_sample_drop =
        sample_valid &&
        (
            fifo_full ||
            fft_path_fault
        );


    assign fft_protocol_error =
        fft_tlast_unexpected ||
        fft_tlast_missing;


    assign fft_peak_valid =
        raw_fft_peak_valid &&
        !fft_path_fault &&
        !fft_sample_drop &&
        !fifo_underflow &&
        !fft_protocol_error;


    // =========================================================
    // Error event bitmap
    // =========================================================

    wire spi_protocol_error;


    assign error_events = {
        21'd0,

        // bit 10
        (
            (|metric_lane_valid) &&
            !(&metric_lane_valid)
        ),

        // bit 9 reserved
        1'b0,

        // bit 8
        adc_init_error,

        // bit 7
        1'b0,

        // bit 6
        (|(metric_lane_error & metric_lane_valid)),

        // bit 5
        fft_protocol_error,

        // bit 4
        spi_protocol_error,

        // bit 3
        (|metric_lane_drop),

        // bit 2
        fifo_underflow,

        // bit 1
        fft_sample_drop,

        // bit 0
        (spi_overrun && adc_init_done)
    };


    // =========================================================
    // Snapshot data
    // =========================================================

    // Each channel:
    //
    // {mean32, rms32, vpp32}
    //
    wire [383:0] metric_packet;


    assign metric_packet[0 +: 96] = {
        {{8{mean0[23]}}, mean0},
        {8'd0, rms0},
        {7'd0, vpp0}
    };


    assign metric_packet[96 +: 96] = {
        {{8{mean1[23]}}, mean1},
        {8'd0, rms1},
        {7'd0, vpp1}
    };


    assign metric_packet[192 +: 96] = {
        {{8{mean2[23]}}, mean2},
        {8'd0, rms2},
        {7'd0, vpp2}
    };


    assign metric_packet[288 +: 96] = {
        {{8{mean3[23]}}, mean3},
        {8'd0, rms3},
        {7'd0, vpp3}
    };


    // =========================================================
    // FFT snapshot packet
    //
    // Each:
    //
    // {mag_hi32, mag_lo32, bin32, frequency32}
    // =========================================================

    wire [511:0] fft_packet;


    assign fft_packet[0 +: 128] = {
        {31'd0, peak_mag0[32]},
        peak_mag0[31:0],
        {21'd0, peak_bin0},
        freq_hz0
    };


    assign fft_packet[128 +: 128] = {
        {31'd0, peak_mag1[32]},
        peak_mag1[31:0],
        {21'd0, peak_bin1},
        freq_hz1
    };


    assign fft_packet[256 +: 128] = {
        {31'd0, peak_mag2[32]},
        peak_mag2[31:0],
        {21'd0, peak_bin2},
        freq_hz2
    };


    assign fft_packet[384 +: 128] = {
        {31'd0, peak_mag3[32]},
        peak_mag3[31:0],
        {21'd0, peak_bin3},
        freq_hz3
    };


    // =========================================================
    // P0-3: low-pass + decimation for raw readout
    //
    //   native 16 kSPS -> FIR 257-tap (fc=890 Hz, anti-alias)
    //                  -> /8 -> 2 kSPS raw frames to MCU
    //
    // decim_lpf_4ch does the anti-alias + decimation; raw_fifo_4ch
    // (DECIMATION=1) only buffers the decimated frames.
    // Each frame: 4 channels x 24-bit, sign-extended to 32-bit
    // =========================================================

    wire signed [23:0] decim_ch0;
    wire signed [23:0] decim_ch1;
    wire signed [23:0] decim_ch2;
    wire signed [23:0] decim_ch3;
    wire               decim_out_valid;

    decim_lpf_4ch #(
        .TAPS       (257),
        .DECIM      (8),
        .COEFF_FILE ("fir_coeff_ads131_16k_2k.mem")
    ) u_decim_lpf (
        .clk         (clk_50m),
        .rst_n       (core_rst_n),
        .sample_valid(sample_valid),
        .ch0_in      (ch0),
        .ch1_in      (ch1),
        .ch2_in      (ch2),
        .ch3_in      (ch3),
        .ch0_out     (decim_ch0),
        .ch1_out     (decim_ch1),
        .ch2_out     (decim_ch2),
        .ch3_out     (decim_ch3),
        .out_valid   (decim_out_valid)
    );

    // 257 FIR taps require 33 decimated outputs before history is full.
    reg [5:0] screen_warmup;
    always @(posedge clk_50m) begin
        if (!core_rst_n || !stream_enable)
            screen_warmup <= 0;
        else if (decim_out_valid && screen_warmup < 33)
            screen_warmup <= screen_warmup + 1'b1;
    end
    assign screen_valid = core_rst_n && stream_enable &&
                          decim_out_valid && screen_warmup == 33;
    assign screen_sample = {decim_ch3, decim_ch2, decim_ch1, decim_ch0};

    wire [31:0] raw_ch0_out;
    wire [31:0] raw_ch1_out;
    wire [31:0] raw_ch2_out;
    wire [31:0] raw_ch3_out;
    wire [31:0] raw_fifo_level_out;
    wire [31:0] raw_dropped_out;
    wire [31:0] raw_decimation_out;
    wire [31:0] raw_frame_number_out;

    raw_fifo_4ch #(
        .DECIMATION(1)
    ) u_raw_fifo (
        .clk            (clk_50m),
        .rst_n          (core_rst_n),
        .sample_valid   (decim_out_valid),
        .ch0            (decim_ch0),
        .ch1            (decim_ch1),
        .ch2            (decim_ch2),
        .ch3            (decim_ch3),
        .reg_rd_strobe  (reg_rd_strobe),
        .reg_rd_addr    (reg_rd_addr),
        .raw_ch0        (raw_ch0_out),
        .raw_ch1        (raw_ch1_out),
        .raw_ch2        (raw_ch2_out),
        .raw_ch3        (raw_ch3_out),
        .raw_fifo_level (raw_fifo_level_out),
        .raw_dropped    (raw_dropped_out),
        .raw_decimation (raw_decimation_out),
        .raw_frame_number(raw_frame_number_out)
    );


    // =========================================================
    // MCU register interface
    // =========================================================

    wire [7:0] reg_rd_addr;
    wire [7:0] reg_wr_addr;

    wire [31:0] reg_rd_data;
    wire [31:0] reg_wr_data;

    wire reg_wr_en;
    wire reg_rd_strobe;

    wire spi_frame_start;
    wire spi_frame_active;

    wire snapshot_hold;
    wire snapshot_request;

    wire snapshot_capture;


    assign snapshot_capture =
        snapshot_request ||
        (
            spi_frame_start &&
            !snapshot_hold
        );


    // =========================================================
    // Snapshot outputs
    // =========================================================

    wire [383:0] snapshot_metrics;
    wire [511:0] snapshot_fft;

    wire [31:0] snapshot_seq;
    wire [31:0] snapshot_metrics_seq;
    wire [31:0] snapshot_fft_seq;

    wire [31:0] snapshot_sample_count;

    wire [31:0] live_metrics_seq;
    wire [31:0] live_fft_seq;

    wire [1:0] snapshot_present;
    wire [1:0] live_present;


    // =========================================================
    // MCU SPI slave
    // =========================================================

    mcu_spi_slave_rw #(
        .LEGACY_READ_ENABLE(1)
    ) u_mcu_spi (
        .clk(
            clk_50m
        ),

        .rst_n(
            core_rst_n
        ),

        .spi_cs_n(
            mcu_cs_n
        ),

        .spi_sclk(
            mcu_sclk
        ),

        .spi_mosi(
            mcu_mosi
        ),

        .spi_miso(
            mcu_miso
        ),

        .spi_miso_oe(
            mcu_miso_oe
        ),

        .reg_rd_addr(
            reg_rd_addr
        ),

        .reg_rd_data(
            reg_rd_data
        ),

        .reg_rd_strobe(
            reg_rd_strobe
        ),

        .reg_wr_addr(
            reg_wr_addr
        ),

        .reg_wr_data(
            reg_wr_data
        ),

        .reg_wr_en(
            reg_wr_en
        ),

        .frame_start(
            spi_frame_start
        ),

        .frame_end(),

        .frame_active(
            spi_frame_active
        ),

        .protocol_error(
            spi_protocol_error
        )
    );


    // =========================================================
    // Snapshot
    // =========================================================

    result_snapshot u_snapshot (
        .clk(
            clk_50m
        ),

        .rst_n(
            core_rst_n
        ),

        .capture(
            snapshot_capture
        ),

        .metrics_valid(
            metrics_valid
        ),

        .metrics_data(
            metric_packet
        ),

        .fft_valid(
            fft_peak_valid
        ),

        .fft_data(
            fft_packet
        ),

        .sample_count(
            sample_count
        ),

        .snapshot_metrics(
            snapshot_metrics
        ),

        .snapshot_fft(
            snapshot_fft
        ),

        .snapshot_seq(
            snapshot_seq
        ),

        .snapshot_metrics_seq(
            snapshot_metrics_seq
        ),

        .snapshot_fft_seq(
            snapshot_fft_seq
        ),

        .snapshot_sample_count(
            snapshot_sample_count
        ),

        .snapshot_present(
            snapshot_present
        ),

        .live_metrics_seq(
            live_metrics_seq
        ),

        .live_fft_seq(
            live_fft_seq
        ),

        .live_present(
            live_present
        )
    );


    // =========================================================
    // Register map
    // =========================================================

    status_register u_registers (
        .clk(
            clk_50m
        ),

        .rst_n(
            core_rst_n
        ),

        .reg_rd_addr(
            reg_rd_addr
        ),

        .reg_rd_data(
            reg_rd_data
        ),

        .reg_rd_strobe(
            reg_rd_strobe
        ),

        .reg_wr_addr(
            reg_wr_addr
        ),

        .reg_wr_data(
            reg_wr_data
        ),

        .reg_wr_en(
            reg_wr_en
        ),

        .snapshot_metrics(
            snapshot_metrics
        ),

        .snapshot_fft(
            snapshot_fft
        ),

        .snapshot_seq(
            snapshot_seq
        ),

        .snapshot_metrics_seq(
            snapshot_metrics_seq
        ),

        .snapshot_fft_seq(
            snapshot_fft_seq
        ),

        .snapshot_sample_count(
            snapshot_sample_count
        ),

        .snapshot_present(
            snapshot_present
        ),

        .live_metrics_seq(
            live_metrics_seq
        ),

        .live_fft_seq(
            live_fft_seq
        ),

        .live_present(
            live_present
        ),

        .adc_init_done(
            adc_init_done
        ),

        .adc_init_error(
            adc_init_error
        ),

        .fifo_full(
            fifo_full
        ),

        .fifo_level(
            fifo_level
        ),

        .fft_configured(
            fft_cfg_sent
        ),

        .fft_path_fault(
            fft_path_fault
        ),

        .spi_active(
            spi_frame_active
        ),

        .adc_status_word(
            status_word
        ),

        .adc_crc_word(
            crc_word
        ),

        .error_events(
            error_events
        ),

        // Raw ADC data
        .raw_ch0        (raw_ch0_out),
        .raw_ch1        (raw_ch1_out),
        .raw_ch2        (raw_ch2_out),
        .raw_ch3        (raw_ch3_out),
        .raw_fifo_level (raw_fifo_level_out),
        .raw_dropped    (raw_dropped_out),
        .raw_decimation (raw_decimation_out),
        .raw_frame_number(raw_frame_number_out),

        // P0-2 / P0-4 / P0-5 diagnostics
        .cfg_id         (cfg_id),
        .fs_hz_actual   (fs_hz_actual),
        .gain1_readback (gain1_readback),
        .gain2_readback (gain2_readback),
        .cfg_readback   (cfg_readback),
        .crc_computed   (crc_computed),
        .crc_err        (crc_err),
        .crc_err_count  (crc_err_count),
        .drdy_event_count(drdy_event_count),
        .rx_frame_count (rx_frame_count),
        .overrun_count  (overrun_count),
        .sample_seq     (sample_seq),

        .snapshot_hold(
            snapshot_hold
        ),

        .snapshot_request(
            snapshot_request
        ),

        .sticky_errors(),

        .irq(
            mcu_irq
        ),

        .clear_stream_counters(
            clear_stream_counters
        )
    );

endmodule

`default_nettype wire
