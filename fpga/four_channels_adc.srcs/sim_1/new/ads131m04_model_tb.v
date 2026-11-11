`timescale 1ns/1ps
`default_nettype none

// Behavioral ADS131M04 model for SPI/initialization verification.
// SPI mode 1: slave updates DOUT on SCLK rising edge; master samples on falling edge.
module ads131m04_model_tb #(
    parameter integer DRDY_PERIOD_NS = 62500,   // 16 kSPS
    parameter integer DRDY_LOW_NS    = 1000
)(
    input  wire       adc_reset_n,
    input  wire       cs_n,
    input  wire       sclk,
    input  wire       din,
    output reg        dout,
    output reg        drdy
);

    reg [143:0] pending_frame;
    reg [143:0] tx_frame;
    reg [143:0] rx_frame;
    reg [7:0]   bit_index;
    reg         frame_active;
    reg [3:0]   ch_counter;

    reg [23:0] sample0;
    reg [23:0] sample1;
    reg [23:0] sample2;
    reg [23:0] sample3;

    initial begin
        dout         = 1'b0;
        drdy         = 1'b1;
        pending_frame= 144'd0;
        tx_frame     = 144'd0;
        rx_frame     = 144'd0;
        bit_index    = 8'd0;
        frame_active = 1'b0;
        ch_counter   = 4'd0;
        sample0      = 24'h001000;
        sample1      = 24'h002000;
        sample2      = 24'h003000;
        sample3      = 24'h004000;
    end

    // ADC RESET: while low, return to reset state.
    always @(negedge adc_reset_n) begin
        pending_frame <= 144'd0;
        tx_frame      <= 144'd0;
        rx_frame      <= 144'd0;
        bit_index     <= 8'd0;
        frame_active  <= 1'b0;
        dout          <= 1'b0;
        drdy          <= 1'b1;
        ch_counter    <= 4'd0;
    end

    // DRDY periodic generator. Starts after RESET release.
    initial begin
        wait (adc_reset_n === 1'b1);
        #10000;
        forever begin
            if (!adc_reset_n) begin
                @(posedge adc_reset_n);
                #10000;
            end
            drdy = 1'b0;
            #DRDY_LOW_NS;
            drdy = 1'b1;
            #(DRDY_PERIOD_NS - DRDY_LOW_NS);
        end
    end

    // Frame start. Current pending_frame is returned in this frame.
    always @(negedge cs_n) begin
        if (adc_reset_n) begin
            frame_active = 1'b1;
            bit_index    = 8'd0;
            rx_frame     = 144'd0;
            tx_frame     = pending_frame;
            dout         = 1'b0;
        end
    end

    // Mode-1 slave: change DOUT on rising edge; FPGA samples it on falling edge.
    always @(posedge sclk) begin
        if (frame_active && !cs_n && adc_reset_n) begin
            dout <= tx_frame[143-bit_index];
            if (bit_index < 8'd143)
                bit_index <= bit_index + 1'b1;
        end
    end

    // Capture FPGA DIN on falling edge. At frame end, decode command.
    always @(negedge sclk) begin
        if (frame_active && !cs_n && adc_reset_n) begin
            rx_frame <= {rx_frame[142:0], din};
        end
    end

    always @(posedge cs_n) begin
        if (frame_active) begin
            frame_active = 1'b0;

            // WREG CLOCK: command 0x6180, data 0x0F06.
            if (rx_frame[143:120] == 24'h618000 &&
                rx_frame[119:96]  == 24'h0F0600) begin
                // WREG response is returned in the following frame.
                pending_frame <= 144'h418000_000000_000000_000000_000000_000000;
            end
            else if (rx_frame[143:120] == 24'h000000) begin
                // NULL consumes the pending WREG response. Then prepare
                // deterministic ADC sample data for subsequent stream frames.
                pending_frame <= {
                    24'h000000,
                    sample0,
                    sample1,
                    sample2,
                    sample3,
                    24'h000000
                };
            end
            else begin
                // Normal stream: next frame has a deterministic ramp.
                sample0 <= sample0 + 24'h000100;
                sample1 <= sample1 + 24'h000100;
                sample2 <= sample2 + 24'h000100;
                sample3 <= sample3 + 24'h000100;
                pending_frame <= {
                    24'h000000,
                    sample0 + 24'h000100,
                    sample1 + 24'h000100,
                    sample2 + 24'h000100,
                    sample3 + 24'h000100,
                    24'h000000
                };
            end
        end
    end

endmodule

`default_nettype wire
