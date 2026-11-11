`timescale 1ns / 1ps
`default_nettype none

/*
 * ads131m04_spi.v
 *
 * ADS131M04 SPI master, mode 1 (CPOL=0, CPHA=1), 144-bit frames.
 *
 * Standard Mode 1 master behavior:
 *   - DIN updated on SCLK rising edge (first edge)
 *   - DOUT sampled on SCLK falling edge (second edge)
 *   - First bit of DIN is already on the bus before the first SCLK rising edge
 *
 * CLK = 50 MHz.
 * Command SCLK = 500 kHz (slow for reliable bring-up)
 * Stream SCLK = 4 MHz.
 *
 * Two transaction types:
 *   - Command: started by cmd_start; TX frame is cmd_tx_frame; response is
 *     captured into cmd_rx_frame and cmd_done pulses when CS is released.
 *   - Stream: started automatically on a DRDY falling edge while
 *     stream_enable is asserted; TX is all zeros; the decoded 4 x 24-bit
 *     channels are presented on ch0_data..ch3_data with sample_valid.
 *
 * NOTE: this file is the validated SPI driver and must stay untouched by
 * acquisition-diagnostic changes. CRC verification, drop counting and
 * sampling-rate metadata are implemented at core-top level (daq_core_top.v),
 * never inside this module.
 */
module ads131m04_spi #(
    parameter integer CLK_FREQ         = 50_000_000,
    parameter integer CMD_SCLK_FREQ    = 500_000,
    parameter integer STREAM_SCLK_FREQ = 4_000_000
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         drdy,
    input  wire         dout,

    output reg          cs_n,
    output reg          sclk,
    output reg          din,

    input  wire         stream_enable,

    input  wire         cmd_start,
    input  wire [143:0]  cmd_tx_frame,
    output reg [143:0]  cmd_rx_frame,
    output reg          cmd_done,

    output reg signed [23:0] ch0_data,
    output reg signed [23:0] ch1_data,
    output reg signed [23:0] ch2_data,
    output reg signed [23:0] ch3_data,
    output reg [23:0]        status_word,
    output reg [23:0]        crc_word,
    output reg               sample_valid,

    output reg busy,
    output reg overrun,

    output wire drdy_event,

    // Debug: latched first word of TX frame (captured at cmd_start)
    output reg [23:0] debug_tx_word0
);

    // Divider: half period = CLK_FREQ / (2 * SCLK_FREQ)
    localparam integer CMD_HALF_DIV =
        (CLK_FREQ / (CMD_SCLK_FREQ * 2));
    localparam integer STREAM_HALF_DIV =
        (CLK_FREQ / (STREAM_SCLK_FREQ * 2));
    localparam integer MAX_HALF_DIV =
        (CMD_HALF_DIV > STREAM_HALF_DIV) ? CMD_HALF_DIV : STREAM_HALF_DIV;
    localparam integer DIV_W =
        (MAX_HALF_DIV <= 1) ? 1 : $clog2(MAX_HALF_DIV + 1);

    reg [DIV_W-1:0] div_cnt;
    reg [DIV_W-1:0] half_div;

    reg [7:0]   bit_cnt;
    reg [143:0] tx_shift;
    reg [143:0] rx_shift;
    reg         transaction_is_stream;
    reg         finish_pending;

    // Minimum CS high time between frames
    localparam integer CS_HOLD_CYCLES = 200;
    reg [7:0] cs_hold_cnt;

    (* ASYNC_REG = "TRUE" *) reg drdy_meta;
    (* ASYNC_REG = "TRUE" *) reg drdy_sync;
    reg drdy_d;

    wire drdy_fall = drdy_d & ~drdy_sync;
    assign drdy_event = drdy_fall;

    wire half_tick = (div_cnt >= half_div - 1'b1);
    wire [143:0] rx_next = {rx_shift[142:0], dout};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n <= 1'b1;
            sclk <= 1'b0;
            din  <= 1'b0;

            div_cnt <= {DIV_W{1'b0}};
            half_div <= CMD_HALF_DIV[DIV_W-1:0];
            bit_cnt <= 8'd0;
            cs_hold_cnt <= 8'd0;

            tx_shift <= 144'd0;
            rx_shift <= 144'd0;
            transaction_is_stream <= 1'b0;
            finish_pending <= 1'b0;

            busy <= 1'b0;
            overrun <= 1'b0;

            cmd_rx_frame <= 144'd0;
            cmd_done <= 1'b0;

            debug_tx_word0 <= 24'd0;

            ch0_data <= 24'sd0;
            ch1_data <= 24'sd0;
            ch2_data <= 24'sd0;
            ch3_data <= 24'sd0;
            status_word <= 24'd0;
            crc_word <= 24'd0;
            sample_valid <= 1'b0;
        end
        else begin
            cmd_done <= 1'b0;
            sample_valid <= 1'b0;

            if (drdy_fall) begin
                if (busy && transaction_is_stream)
                    overrun <= 1'b1;
            end

            // ----------------------------------------------------------
            // Frame completion path (CS held high, then released)
            // ----------------------------------------------------------
            if (finish_pending) begin
                cs_n <= 1'b1;
                sclk <= 1'b0;
                din  <= 1'b0;
                finish_pending <= 1'b0;
                div_cnt <= {DIV_W{1'b0}};
                cs_hold_cnt <= CS_HOLD_CYCLES[7:0];
            end
            else if (cs_hold_cnt > 8'd0) begin
                cs_hold_cnt <= cs_hold_cnt - 1'b1;
                if (cs_hold_cnt == 8'd1) begin
                    busy <= 1'b0;
                    cmd_rx_frame <= rx_shift;
                    if (transaction_is_stream) begin
                        status_word <= rx_shift[143:120];
                        ch0_data <= rx_shift[119:96];
                        ch1_data <= rx_shift[95:72];
                        ch2_data <= rx_shift[71:48];
                        ch3_data <= rx_shift[47:24];
                        crc_word <= rx_shift[23:0];
                        sample_valid <= 1'b1;
                    end
                    else begin
                        cmd_done <= 1'b1;
                    end
                end
            end
            // ----------------------------------------------------------
            // Idle: start a new transaction
            // ----------------------------------------------------------
            else if (!busy) begin
                cs_n <= 1'b1;
                sclk <= 1'b0;
                din  <= 1'b0;
                div_cnt <= {DIV_W{1'b0}};
                bit_cnt <= 8'd0;

                if (cmd_start) begin
                    cs_n <= 1'b0;
                    busy <= 1'b1;
                    transaction_is_stream <= 1'b0;
                    half_div <= CMD_HALF_DIV[DIV_W-1:0];
                    tx_shift <= cmd_tx_frame;
                    rx_shift <= 144'd0;
                    finish_pending <= 1'b0;
                    // First bit on DIN BEFORE first SCLK rising edge
                    din <= cmd_tx_frame[143];
                    debug_tx_word0 <= cmd_tx_frame[143:120];
                    overrun <= 1'b0;
                end
                else if (stream_enable && drdy_fall) begin
                    cs_n <= 1'b0;
                    busy <= 1'b1;
                    transaction_is_stream <= 1'b1;
                    half_div <= STREAM_HALF_DIV[DIV_W-1:0];
                    tx_shift <= 144'd0;
                    rx_shift <= 144'd0;
                    finish_pending <= 1'b0;
                    din <= 1'b0;
                    overrun <= 1'b0;
                end
                else begin
                    busy <= 1'b0;
                end
            end
            // ----------------------------------------------------------
            // Transaction in progress: half-period tick
            // ----------------------------------------------------------
            else if (half_tick) begin
                div_cnt <= {DIV_W{1'b0}};

                if (!sclk) begin
                    // ------------------------------------------------
                    // RISING edge — SPI Mode 1
                    //
                    // ADS131M04:
                    //   - DOUT changes on this edge
                    //   - DIN will be sampled on the next FALLING edge
                    //
                    // FPGA:
                    //   - Update DIN on this edge (standard Mode 1 master)
                    //     so DIN is stable throughout the high period
                    //     and the ADC can safely sample it on the
                    //     next falling edge.
                    // ------------------------------------------------
                    sclk <= 1'b1;

                    if (bit_cnt == 8'd0) begin
                        // First bit was already set at CS low.
                        // Keep it stable — don't update yet.
                    end
                    else begin
                        din <= tx_shift[143];
                    end
                end
                else begin
                    // ------------------------------------------------
                    // FALLING edge — SPI Mode 1
                    //
                    // ADS131M04:
                    //   - Samples DIN on this edge
                    //   - DOUT was set up on the previous rising edge
                    //
                    // FPGA:
                    //   - Samples DOUT on this edge
                    //   - Shifts TX register
                    // ------------------------------------------------
                    sclk     <= 1'b0;
                    rx_shift <= rx_next;

                    if (bit_cnt == 8'd143) begin
                        finish_pending <= 1'b1;
                    end
                    else begin
                        bit_cnt  <= bit_cnt + 1'b1;
                        tx_shift <= {tx_shift[142:0], 1'b0};
                    end
                end
            end
            else begin
                div_cnt <= div_cnt + 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
