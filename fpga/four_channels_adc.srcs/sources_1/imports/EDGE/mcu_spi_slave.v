`timescale 1ns / 1ps
/*
 * mcu_spi_slave.v
 *
 * MCU <-> FPGA measurement readout interface.
 *
 * MCU is SPI master, FPGA is SPI slave.
 * SPI mode 0: CPOL=0, CPHA=0, MSB first.
 *
 * This implementation synchronizes MCU SPI into clk=50 MHz and detects
 * edges there. Keep MCU SCLK <= 2 MHz for comfortable timing margin.
 *
 * Transaction format:
 *   CS low
 *   byte 0 : register address from MCU
 *   bytes 1..4 : FPGA returns one 32-bit value, MSB first
 *   CS high
 *
 * Register map:
 *   0x00 status      bit0=adc_init_done, bit1=adc_init_error
 *   0x10 CH0 VPP     ADC codes
 *   0x11 CH0 RMS     ADC codes
 *   0x12 CH0 FREQ    Hz
 *   0x20 CH1 VPP
 *   0x21 CH1 RMS
 *   0x22 CH1 FREQ
 *   0x30 CH2 VPP
 *   0x31 CH2 RMS
 *   0x32 CH2 FREQ
 *   0x40 CH3 VPP
 *   0x41 CH3 RMS
 *   0x42 CH3 FREQ
 *   0x7F signature   0xA5A71304
 */
module mcu_spi_slave(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        spi_cs_n,
    input  wire        spi_sclk,
    input  wire        spi_mosi,
    output reg         spi_miso,

    input  wire        adc_init_done,
    input  wire        adc_init_error,

    input  wire [24:0] vpp0,
    input  wire [24:0] vpp1,
    input  wire [24:0] vpp2,
    input  wire [24:0] vpp3,

    input  wire [23:0] rms0,
    input  wire [23:0] rms1,
    input  wire [23:0] rms2,
    input  wire [23:0] rms3,

    input  wire [31:0] freq_hz0,
    input  wire [31:0] freq_hz1,
    input  wire [31:0] freq_hz2,
    input  wire [31:0] freq_hz3
);

(* ASYNC_REG = "TRUE" *) reg [2:0] cs_sync;
(* ASYNC_REG = "TRUE" *) reg [2:0] sclk_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] mosi_sync;

wire cs_active = ~cs_sync[2];
wire sclk_rise = (sclk_sync[2:1] == 2'b01);
wire sclk_fall = (sclk_sync[2:1] == 2'b10);

reg [7:0] addr_shift;
reg [2:0] addr_bit_count;
reg       read_phase;
reg [31:0] tx_shift;

wire [7:0] addr_candidate = {addr_shift[6:0], mosi_sync[1]};

function [31:0] read_register;
    input [7:0] addr;
    begin
        case (addr)
            8'h00: read_register = {30'd0, adc_init_error, adc_init_done};

            8'h10: read_register = {7'd0, vpp0};
            8'h11: read_register = {8'd0, rms0};
            8'h12: read_register = freq_hz0;

            8'h20: read_register = {7'd0, vpp1};
            8'h21: read_register = {8'd0, rms1};
            8'h22: read_register = freq_hz1;

            8'h30: read_register = {7'd0, vpp2};
            8'h31: read_register = {8'd0, rms2};
            8'h32: read_register = freq_hz2;

            8'h40: read_register = {7'd0, vpp3};
            8'h41: read_register = {8'd0, rms3};
            8'h42: read_register = freq_hz3;

            8'h7F: read_register = 32'hA5A71304;
            default: read_register = 32'd0;
        endcase
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cs_sync   <= 3'b111;
        sclk_sync <= 3'b000;
        mosi_sync <= 2'b00;
    end else begin
        cs_sync   <= {cs_sync[1:0], spi_cs_n};
        sclk_sync <= {sclk_sync[1:0], spi_sclk};
        mosi_sync <= {mosi_sync[0], spi_mosi};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        addr_shift     <= 8'd0;
        addr_bit_count <= 3'd0;
        read_phase     <= 1'b0;
        tx_shift       <= 32'd0;
        spi_miso       <= 1'b0;
    end else begin
        if (!cs_active) begin
            addr_shift     <= 8'd0;
            addr_bit_count <= 3'd0;
            read_phase     <= 1'b0;
            tx_shift       <= 32'd0;
            spi_miso       <= 1'b0;
        end else begin
            /* Address is sampled on SPI rising edges. */
            if (!read_phase && sclk_rise) begin
                addr_shift <= addr_candidate;

                if (addr_bit_count == 3'd7) begin
                    tx_shift       <= read_register(addr_candidate);
                    addr_bit_count <= 3'd0;
                    read_phase     <= 1'b1;
                end else begin
                    addr_bit_count <= addr_bit_count + 1'b1;
                end
            end

            /*
             * Mode 0: change MISO on falling edges so MCU samples a stable
             * value on the following rising edge.
             */
            if (read_phase && sclk_fall) begin
                spi_miso <= tx_shift[31];
                tx_shift <= {tx_shift[30:0], 1'b0};
            end
        end
    end
end

endmodule

`timescale 1ns / 1ps
`default_nettype none
/*
 * MCU SPI slave, mode 0, MSB first, single 50 MHz processing clock.
 * SCLK is oversampled, not used as an FPGA fabric clock.
 * Board-level timing contract at 50 MHz:
 *   SCLK <= 2 MHz; each high/low >= 240 ns;
 *   CS setup/hold/high gap >= 200 ns; MOSI stable around SCLK rising.
 * A combinational register read mux must settle before the next detected
 * falling edge. No synchronous-RAM latency is supported by this interface.
 *
 * Extended READ : CS low, A0, ADDR, 4*K dummy bytes, CS high.
 * Extended WRITE: CS low, A1, ADDR, 4*K data bytes,  CS high.
 * Each register is 32 bits, big endian, and address increments per WORD.
 * No dummy byte between address and read data.
 * Optional legacy: first byte 00..7F means a read address, as in the upload.
 * Writes commit only after ALL 32 data bits. Partial writes never commit.
 * Burst writes are per-word transactions, not an all-or-nothing burst.
 *
 * spi_miso is a data bit, NOT internally tri-stated. Connect spi_miso_oe
 * to a top-level OBUFT or 'assign pin = oe ? data : 1'bz'. OE uses raw CS
 * for prompt bus release; all protocol/register logic uses synchronized CS.
 */
module mcu_spi_slave_rw #(
    parameter integer LEGACY_READ_ENABLE = 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        spi_cs_n,
    input  wire        spi_sclk,
    input  wire        spi_mosi,
    output reg         spi_miso,
    output wire        spi_miso_oe,
    output wire [7:0]  reg_rd_addr,
    input  wire [31:0] reg_rd_data,
    output reg         reg_rd_strobe,
    output reg  [7:0]  reg_wr_addr,
    output reg  [31:0] reg_wr_data,
    output reg         reg_wr_en,
    output reg         frame_start,
    output reg         frame_end,
    output wire        frame_active,
    output reg         protocol_error
);
    // First two stages are metastability synchronizers; third is history.
    (* ASYNC_REG = "TRUE" *) reg [1:0] cs_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sclk_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] mosi_sync;
    reg cs_delayed, sclk_delayed;
    wire cs_begin = cs_delayed && !cs_sync[1];
    wire cs_end = !cs_delayed && cs_sync[1];
    wire sclk_rise = !sclk_delayed && sclk_sync[1];
    wire sclk_fall = sclk_delayed && !sclk_sync[1];
    assign frame_active = !cs_sync[1];
    assign spi_miso_oe = rst_n && !spi_cs_n;

    localparam [2:0] CMD=0, ADDR=1, READ_DATA=2, WRITE_DATA=3, IGNORE=4;
    reg [2:0] state;
    reg [5:0] bit_count;
    reg [7:0] byte_shift;
    reg [7:0] address;
    reg write_command;
    reg [31:0] rx_shift, tx_shift;
    reg load_word;
    wire [7:0] received_byte = {byte_shift[6:0], mosi_sync[1]};
    assign reg_rd_addr = address;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_sync <= 2'b11; sclk_sync <= 0; mosi_sync <= 0;
            cs_delayed <= 1'b1; sclk_delayed <= 1'b0;
        end else begin
            cs_sync <= {cs_sync[0], spi_cs_n};
            sclk_sync <= {sclk_sync[0], spi_sclk};
            mosi_sync <= {mosi_sync[0], spi_mosi};
            cs_delayed <= cs_sync[1];
            sclk_delayed <= sclk_sync[1];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= CMD; bit_count <= 0; byte_shift <= 0;
            address <= 0; write_command <= 0;
            rx_shift <= 0; tx_shift <= 0; load_word <= 0;
            spi_miso <= 0;
            reg_rd_strobe <= 0; reg_wr_addr <= 0;
            reg_wr_data <= 0; reg_wr_en <= 0;
            frame_start <= 0; frame_end <= 0; protocol_error <= 0;
        end else begin
            reg_rd_strobe <= 1'b0; reg_wr_en <= 1'b0;
            frame_start <= 1'b0; frame_end <= 1'b0; protocol_error <= 1'b0;
            if (cs_begin) begin
                state <= CMD; bit_count <= 0; byte_shift <= 0;
                address <= 0; write_command <= 0;
                rx_shift <= 0; tx_shift <= 0; load_word <= 0;
                spi_miso <= 0;
                frame_start <= 1'b1;
            end else if (cs_end) begin
                if ((state == ADDR) ||
                    ((state == CMD || state == WRITE_DATA) && bit_count != 0))
                    protocol_error <= 1'b1;
                // Ending a partial READ is allowed and has no side effects.
                state <= CMD; bit_count <= 0; load_word <= 0;
                spi_miso <= 0;
                frame_end <= 1'b1;
            end else if (frame_active) begin
                if (sclk_rise) begin
                    case (state)
                        CMD, ADDR: begin
                            byte_shift <= received_byte;
                            if (bit_count == 6'd7) begin
                                bit_count <= 0;
                                if (state == ADDR) begin
                                    address <= received_byte;
                                    if (write_command) begin
                                        state <= WRITE_DATA;
                                    end else begin
                                        // --- MODE-0 FIRST-BIT FIX ---
                                        // Place MSB of the read data on MISO
                                        // on the sclk_rise that latches the
                                        // last address bit. MCU Mode-0 samples
                                        // on SCLK rising edge, so bit 31 must
                                        // be stable before the first data-
                                        // phase SCLK rising edge arrives.
                                        state <= READ_DATA;
                                        load_word <= 1'b1;
                                        spi_miso <= reg_rd_data[31];
                                        tx_shift <= {reg_rd_data[30:0], 1'b0};
                                        reg_rd_strobe <= 1'b1;
                                    end
                                end else if (received_byte == 8'hA0 ||
                                             received_byte == 8'hA1) begin
                                    write_command <= received_byte[0];
                                    state <= ADDR;
                                end else if (LEGACY_READ_ENABLE != 0 &&
                                             !received_byte[7]) begin
                                    // Legacy direct read: same fix
                                    address <= received_byte;
                                    state <= READ_DATA;
                                    load_word <= 1'b1;
                                    spi_miso <= reg_rd_data[31];
                                    tx_shift <= {reg_rd_data[30:0], 1'b0};
                                    reg_rd_strobe <= 1'b1;
                                end else begin
                                    state <= IGNORE;
                                    protocol_error <= 1'b1;
                                end
                            end else begin
                                bit_count <= bit_count + 1'b1;
                            end
                        end
                        READ_DATA: begin
                            // bit_count == 0 means MSB was just sent (strobe
                            // already fired in ADDR state transition).
                            // For subsequent words loaded via load_word on
                            // sclk_fall, we strobe here too on bit 0 cycle.
                            if (bit_count == 0 && !load_word) reg_rd_strobe <= 1'b1;
                            if (bit_count == 6'd31) begin
                                bit_count <= 0;
                                address <= address + 8'd1;
                                load_word <= 1'b1;
                            end else bit_count <= bit_count + 1'b1;
                        end
                        WRITE_DATA: begin
                            rx_shift <= {rx_shift[30:0], mosi_sync[1]};
                            if (bit_count == 6'd31) begin
                                reg_wr_addr <= address;
                                reg_wr_data <= {rx_shift[30:0], mosi_sync[1]};
                                reg_wr_en <= 1'b1;
                                address <= address + 8'd1;
                                bit_count <= 0;
                            end else bit_count <= bit_count + 1'b1;
                        end
                        default: begin end
                    endcase
                end
                if (sclk_fall && state == READ_DATA) begin
                    if (load_word) begin
                        spi_miso <= reg_rd_data[31];
                        tx_shift <= {reg_rd_data[30:0], 1'b0};
                        load_word <= 1'b0;
                    end else begin
                        spi_miso <= tx_shift[31];
                        tx_shift <= {tx_shift[30:0], 1'b0};
                    end
                end
            end
        end
    end
endmodule
`default_nettype wire

`timescale 1ns / 1ps
`default_nettype none
/*
 * Two register banks in ONE clk domain:
 *   latest bank: independently updated by metrics_valid / fft_valid.
 *   read bank:   copied only on capture, stable until the next capture.
 *
 * Assert capture at SPI transaction start, or explicitly while auto capture
 * is disabled. A capture coincident with an update INCLUDES that update.
 * There is no valid/data CDC here; all inputs must be synchronous to clk.
 *
 * Atomic register readout does NOT imply temporal alignment of the metrics
 * and FFT windows. Separate sequence numbers identify their independent
 * update histories. They are versions, not proof of equal sample windows.
 */
module result_snapshot #(
    parameter integer METRICS_W = 384,
    parameter integer FFT_W = 512
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 capture,
    input  wire                 metrics_valid,
    input  wire [METRICS_W-1:0]  metrics_data,
    input  wire                 fft_valid,
    input  wire [FFT_W-1:0]      fft_data,
    input  wire [31:0]          sample_count,
    output reg  [METRICS_W-1:0]  snapshot_metrics,
    output reg  [FFT_W-1:0]      snapshot_fft,
    output reg  [31:0]          snapshot_seq,
    output reg  [31:0]          snapshot_metrics_seq,
    output reg  [31:0]          snapshot_fft_seq,
    output reg  [31:0]          snapshot_sample_count,
    output reg  [1:0]           snapshot_present,
    output reg  [31:0]          live_metrics_seq,
    output reg  [31:0]          live_fft_seq,
    output reg  [1:0]           live_present
);
    reg [METRICS_W-1:0] latest_metrics;
    reg [FFT_W-1:0] latest_fft;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latest_metrics <= 0; latest_fft <= 0;
            snapshot_metrics <= 0; snapshot_fft <= 0;
            snapshot_seq <= 0; snapshot_metrics_seq <= 0;
            snapshot_fft_seq <= 0; snapshot_sample_count <= 0;
            snapshot_present <= 0;
            live_metrics_seq <= 0; live_fft_seq <= 0; live_present <= 0;
        end else begin
            if (metrics_valid) begin
                latest_metrics <= metrics_data;
                live_metrics_seq <= live_metrics_seq + 32'd1;
                live_present[0] <= 1'b1;
            end
            if (fft_valid) begin
                latest_fft <= fft_data;
                live_fft_seq <= live_fft_seq + 32'd1;
                live_present[1] <= 1'b1;
            end
            if (capture) begin
                snapshot_metrics <= metrics_valid ? metrics_data : latest_metrics;
                snapshot_fft <= fft_valid ? fft_data : latest_fft;
                snapshot_metrics_seq <= live_metrics_seq + {31'd0, metrics_valid};
                snapshot_fft_seq <= live_fft_seq + {31'd0, fft_valid};
                snapshot_sample_count <= sample_count;
                snapshot_seq <= snapshot_seq + 32'd1;
                snapshot_present <= live_present | {fft_valid, metrics_valid};
            end
        end
    end
endmodule
`default_nettype wire

`timescale 1ns / 1ps
`default_nettype none
/*
 * 8-bit WORD-addressed, 32-bit register bank. Read mux is combinational;
 * writes commit on reg_wr_en. All inputs are synchronous to clk.
 * Keep the uploaded legacy addresses: 00 status, 10/11/12 CH0 Vpp/RMS/Hz,
 * 20/21/22 CH1, 30/31/32 CH2, 40/41/42 CH3, 7F signature A5A71304.
 * New controls, diagnostic registers, and mean/bin/magnitude are documented
 * in docs/register_map.md. Unsupported reads return DEADBAAD. Writes to
 * read-only/undefined addresses are ignored and set error bit 7.
 * W1C is set-dominant: a concurrent new event survives a software clear.
 */
module status_register #(
    parameter [31:0] FS_HZ = 32'd16000,     // configured fDATA (16 kSPS)
    parameter [31:0] WINDOW_N = 32'd2048
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  reg_rd_addr,
    output reg  [31:0] reg_rd_data,
    input  wire        reg_rd_strobe,
    input  wire [7:0]  reg_wr_addr,
    input  wire [31:0] reg_wr_data,
    input  wire        reg_wr_en,
    input  wire [383:0] snapshot_metrics,
    input  wire [511:0] snapshot_fft,
    input  wire [31:0] snapshot_seq,
    input  wire [31:0] snapshot_metrics_seq,
    input  wire [31:0] snapshot_fft_seq,
    input  wire [31:0] snapshot_sample_count,
    input  wire [1:0]  snapshot_present,
    input  wire [31:0] live_metrics_seq,
    input  wire [31:0] live_fft_seq,
    input  wire [1:0]  live_present,
    input  wire        adc_init_done,
    input  wire        adc_init_error,
    input  wire        fifo_full,
    input  wire [12:0] fifo_level,
    input  wire        fft_configured,
    input  wire        fft_path_fault,
    input  wire        spi_active,
    input  wire [23:0] adc_status_word,
    input  wire [23:0] adc_crc_word,
    input  wire [31:0] error_events,

    // Raw ADC FIFO interface
    input  wire [31:0] raw_ch0,
    input  wire [31:0] raw_ch1,
    input  wire [31:0] raw_ch2,
    input  wire [31:0] raw_ch3,
    input  wire [31:0] raw_fifo_level,
    input  wire [31:0] raw_dropped,
    input  wire [31:0] raw_decimation,
    input  wire [31:0] raw_frame_number,

    // P0-2 / P0-4 / P0-5 diagnostics
    input  wire [31:0] cfg_id,
    input  wire [31:0] fs_hz_actual,
    input  wire [15:0] gain1_readback,
    input  wire [15:0] gain2_readback,
    input  wire [15:0] cfg_readback,
    input  wire [7:0]  crc_computed,
    input  wire        crc_err,
    input  wire [31:0] crc_err_count,
    input  wire [31:0] drdy_event_count,
    input  wire [31:0] rx_frame_count,
    input  wire [31:0] overrun_count,
    input  wire [31:0] sample_seq,

    output reg         snapshot_hold,
    output reg         snapshot_request,
    output reg  [31:0] sticky_errors,
    output wire        irq,
    output reg         clear_stream_counters
);
    reg [31:0] irq_mask;
    reg [31:0] dropped_fft_samples;
    reg [31:0] dropped_metric_windows;
    reg [31:0] spi_error_count;
    reg [31:0] scratch0, scratch1;
    reg rd_valid;
    reg wr_valid;
    integer ch;
    wire [31:0] clear_mask =
        (reg_wr_en && reg_wr_addr == 8'h04) ? reg_wr_data : 32'd0;
    wire clear_counters = reg_wr_en && reg_wr_addr == 8'h59 && reg_wr_data[0];
    wire access_error = (reg_rd_strobe && !rd_valid) || (reg_wr_en && !wr_valid);
    wire [31:0] new_errors = error_events | (access_error ? 32'h00000080 : 32'd0);
    assign irq = |(sticky_errors & irq_mask);

    function [31:0] sat_inc;
        input [31:0] value;
        begin sat_inc = (&value) ? value : value + 32'd1; end
    endfunction

    always @* begin
        case (reg_wr_addr)
            8'h02, 8'h03, 8'h04, 8'h05, 8'h59, 8'h60, 8'h61: wr_valid = 1'b1;
            default: wr_valid = 1'b0;
        endcase
    end

    always @* begin
        rd_valid = 1'b1;
        reg_rd_data = 0;
        ch = 0;
        case (reg_rd_addr)
            8'h00: begin
                reg_rd_data[0] = adc_init_done;
                reg_rd_data[1] = adc_init_error;
                reg_rd_data[3:2] = live_present;
                reg_rd_data[4] = snapshot_hold;
                reg_rd_data[5] = fifo_full;
                reg_rd_data[6] = fft_configured;
                reg_rd_data[7] = spi_active;
                reg_rd_data[8] = |sticky_errors;
                reg_rd_data[9] = fft_path_fault;
                reg_rd_data[11:10] = snapshot_present;
                reg_rd_data[12] = (live_metrics_seq != snapshot_metrics_seq);
                reg_rd_data[13] = (live_fft_seq != snapshot_fft_seq);
            end
            8'h01: reg_rd_data = 32'h00020000; // register ABI 2.0
            8'h02: reg_rd_data = {31'd0, snapshot_hold};
            8'h03: reg_rd_data = 0; // snapshot command self-clears
            8'h04: reg_rd_data = sticky_errors;
            8'h05: reg_rd_data = irq_mask;
            8'h06: reg_rd_data = snapshot_seq;
            8'h07: reg_rd_data = snapshot_metrics_seq;
            8'h08: reg_rd_data = snapshot_fft_seq;
            8'h09: reg_rd_data = snapshot_sample_count;
            8'h0A: reg_rd_data = FS_HZ;
            8'h0B: reg_rd_data = WINDOW_N;
            8'h0C: reg_rd_data = {19'd0, fifo_level};
            8'h0D: reg_rd_data = dropped_fft_samples;
            8'h0E: reg_rd_data = dropped_metric_windows;
            8'h0F: reg_rd_data = spi_error_count;
            8'h50: reg_rd_data = live_metrics_seq;
            8'h51: reg_rd_data = live_fft_seq;
            8'h52: reg_rd_data = {30'd0, snapshot_present};
            8'h53: reg_rd_data = 32'h0000000F; // four fixed enabled ADC channels
            8'h54: reg_rd_data = {8'd0, adc_status_word};
            8'h55: reg_rd_data = {8'd0, adc_crc_word}; // RAW, NOT CRC-verified
            8'h56: reg_rd_data = {23'd0, crc_computed, crc_err}; // computed CRC + result
            8'h57: reg_rd_data = sample_seq;
            8'h58: reg_rd_data = drdy_event_count;
            8'h59: reg_rd_data = 0;
            8'h5A: reg_rd_data = rx_frame_count;
            8'h5B: reg_rd_data = overrun_count;
            8'h5C: reg_rd_data = crc_err_count;
            8'h60: reg_rd_data = scratch0;
            8'h61: reg_rd_data = scratch1;

            // Raw ADC data registers
            8'h62: reg_rd_data = raw_ch0;
            8'h63: reg_rd_data = raw_ch1;
            8'h64: reg_rd_data = raw_ch2;
            8'h65: reg_rd_data = raw_ch3;
            8'h66: reg_rd_data = raw_fifo_level;
            8'h67: reg_rd_data = raw_dropped;
            8'h68: reg_rd_data = raw_decimation;
            8'h69: reg_rd_data = raw_frame_number;
            8'h6A: reg_rd_data = fs_hz_actual;
            8'h6B: reg_rd_data = cfg_id;
            8'h6C: reg_rd_data = {gain1_readback, gain2_readback};
            8'h6D: reg_rd_data = {16'd0, cfg_readback};

            8'h7F: reg_rd_data = 32'hA5A71304;
            default: begin
                if (reg_rd_addr[7:4] >= 4'h1 && reg_rd_addr[7:4] <= 4'h4 &&
                    reg_rd_addr[3:0] <= 4'h6) begin
                    ch = {28'd0, reg_rd_addr[7:4]} - 1;
                    case (reg_rd_addr[3:0])
                        4'h0: reg_rd_data = snapshot_metrics[ch*96 +: 32];
                        4'h1: reg_rd_data = snapshot_metrics[ch*96+32 +: 32];
                        4'h2: reg_rd_data = snapshot_fft[ch*128 +: 32];
                        4'h3: reg_rd_data = snapshot_metrics[ch*96+64 +: 32];
                        4'h4: reg_rd_data = snapshot_fft[ch*128+32 +: 32];
                        4'h5: reg_rd_data = snapshot_fft[ch*128+64 +: 32];
                        4'h6: reg_rd_data = snapshot_fft[ch*128+96 +: 32];
                        default: reg_rd_data = 0;
                    endcase
                end else begin
                    rd_valid = 1'b0;
                    reg_rd_data = 32'hDEADBAAD;
                end
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapshot_hold <= 0; snapshot_request <= 0;
            clear_stream_counters <= 1'b0;
            sticky_errors <= 0; irq_mask <= 0;
            dropped_fft_samples <= 0; dropped_metric_windows <= 0;
            spi_error_count <= 0; scratch0 <= 0; scratch1 <= 0;
        end else begin
            snapshot_request <= 1'b0;
            clear_stream_counters <= clear_counters;
            sticky_errors <= (sticky_errors & ~clear_mask) | new_errors;
            if (clear_counters) begin
                // A same-cycle event counts as the first event after clear.
                dropped_fft_samples <= {31'd0, error_events[1]};
                dropped_metric_windows <= {31'd0, error_events[3]};
                spi_error_count <= {31'd0, error_events[4]};
            end else begin
                if (error_events[1]) dropped_fft_samples <= sat_inc(dropped_fft_samples);
                if (error_events[3]) dropped_metric_windows <= sat_inc(dropped_metric_windows);
                if (error_events[4]) spi_error_count <= sat_inc(spi_error_count);
            end
            if (reg_wr_en) begin
                case (reg_wr_addr)
                    8'h02: snapshot_hold <= reg_wr_data[0];
                    8'h03: snapshot_request <= reg_wr_data[0];
                    8'h05: irq_mask <= reg_wr_data;
                    8'h60: scratch0 <= reg_wr_data;
                    8'h61: scratch1 <= reg_wr_data;
                    default: begin end
                endcase
            end
        end
    end
endmodule
`default_nettype wire
