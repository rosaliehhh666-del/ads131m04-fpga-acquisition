`timescale 1ns/1ps
`default_nettype none

module tb_ads131m04_init_spi;

    reg clk = 1'b0;
    reg rst_n = 1'b0;

    wire adc_reset_n;
    wire adc_cs_n;
    wire adc_sclk;
    wire adc_din;
    wire adc_dout;
    wire adc_drdy;

    wire spi_busy;
    wire cmd_start;
    wire [143:0] cmd_tx_frame;
    wire [143:0] cmd_rx_frame;
    wire cmd_done;
    wire stream_enable;
    wire adc_init_done;
    wire adc_init_error;

    wire signed [23:0] ch0_data;
    wire signed [23:0] ch1_data;
    wire signed [23:0] ch2_data;
    wire signed [23:0] ch3_data;
    wire [23:0] status_word;
    wire [23:0] crc_word;
    wire sample_valid;
    wire spi_overrun;
    wire drdy_event;

    integer sample_count;
    integer cmd_count;
    integer error_count;
    integer timeout_count;
    reg saw_reset_low;
    reg saw_reset_high;
    reg saw_wreg;
    reg saw_null;

    always #10 clk = ~clk; // 50 MHz

    ads131m04_init #(
        .CLK_FREQ(50_000_000),
        .STARTUP_DELAY_CYCLES(1000) // 20 us in simulation
    ) u_init (
        .clk(clk),
        .rst_n(rst_n),
        .spi_busy(spi_busy),
        .cmd_done(cmd_done),
        .cmd_rx_frame(cmd_rx_frame),
        .drdy_event(drdy_event),
        .cmd_start(cmd_start),
        .cmd_tx_frame(cmd_tx_frame),
        .stream_enable(stream_enable),
        .adc_reset_n(adc_reset_n),
        .init_done(adc_init_done),
        .init_error(adc_init_error)
    );

    ads131m04_spi #(
        .CLK_FREQ(50_000_000),
        .CMD_SCLK_FREQ(2_000_000),
        .STREAM_SCLK_FREQ(4_000_000)
    ) u_spi (
        .clk(clk),
        .rst_n(rst_n),
        .drdy(adc_drdy),
        .dout(adc_dout),
        .cs_n(adc_cs_n),
        .sclk(adc_sclk),
        .din(adc_din),
        .stream_enable(stream_enable),
        .cmd_start(cmd_start),
        .cmd_tx_frame(cmd_tx_frame),
        .cmd_rx_frame(cmd_rx_frame),
        .cmd_done(cmd_done),
        .ch0_data(ch0_data),
        .ch1_data(ch1_data),
        .ch2_data(ch2_data),
        .ch3_data(ch3_data),
        .status_word(status_word),
        .crc_word(crc_word),
        .sample_valid(sample_valid),
        .busy(spi_busy),
        .overrun(spi_overrun),
        .drdy_event(drdy_event)
    );

    tb_ads131m04_model #(
        .DRDY_PERIOD_NS(62500),
        .DRDY_LOW_NS(1000)
    ) u_adc (
        .adc_reset_n(adc_reset_n),
        .cs_n(adc_cs_n),
        .sclk(adc_sclk),
        .din(adc_din),
        .dout(adc_dout),
        .drdy(adc_drdy)
    );

    always @(negedge adc_reset_n) begin
        saw_reset_low = 1'b1;
        $display("[%0t] ADC RESET LOW", $time);
    end

    always @(posedge adc_reset_n) begin
        saw_reset_high = 1'b1;
        $display("[%0t] ADC RESET HIGH", $time);
    end

    always @(posedge cmd_start) begin
        cmd_count = cmd_count + 1;
        if (cmd_tx_frame == 144'h618000_0F0600_000000_000000_000000_000000) begin
            saw_wreg = 1'b1;
            $display("[%0t] WREG CLOCK command detected: %h", $time, cmd_tx_frame);
        end
        else if (cmd_tx_frame == 144'd0) begin
            saw_null = 1'b1;
            $display("[%0t] NULL command detected", $time);
        end
        else begin
            $display("[%0t] Unexpected command: %h", $time, cmd_tx_frame);
            error_count = error_count + 1;
        end
    end

    always @(posedge cmd_done) begin
        $display("[%0t] CMD DONE, RX word0 = %h", $time, cmd_rx_frame[143:120]);
        if (saw_null && cmd_rx_frame[143:128] != 16'h4180) begin
            $display("ERROR: WREG ACK mismatch, got %h", cmd_rx_frame[143:120]);
            error_count = error_count + 1;
        end
    end

    always @(posedge sample_valid) begin
        sample_count = sample_count + 1;
        if (sample_count <= 5)
            $display("[%0t] SAMPLE %0d: CH0=%h CH1=%h CH2=%h CH3=%h STATUS=%h", $time,
                     sample_count, ch0_data, ch1_data, ch2_data, ch3_data, status_word);
    end

    always @(posedge clk) begin
        if (adc_init_error) begin
            $display("[%0t] ERROR: adc_init_error asserted", $time);
            error_count = error_count + 1;
            $finish;
        end
    end

    initial begin
        sample_count = 0;
        cmd_count = 0;
        error_count = 0;
        timeout_count = 0;
        saw_reset_low = 0;
        saw_reset_high = 0;
        saw_wreg = 0;
        saw_null = 0;

        $dumpfile("ads131m04_init_spi.vcd");
        $dumpvars(0, tb_ads131m04_init_spi);

        #100;
        rst_n = 1'b1;
        $display("[%0t] FPGA RESET RELEASED", $time);

        // Run for 1.5 ms: enough for initialization + many 16 kSPS frames.
        #1_500_000;

        $display("\n========== ADS131M04 SIMULATION RESULT ==========");
        $display("RESET low seen      : %0d", saw_reset_low);
        $display("RESET high seen     : %0d", saw_reset_high);
        $display("WREG seen           : %0d", saw_wreg);
        $display("NULL seen           : %0d", saw_null);
        $display("INIT DONE            : %0d", adc_init_done);
        $display("INIT ERROR           : %0d", adc_init_error);
        $display("stream_enable       : %0d", stream_enable);
        $display("samples received    : %0d", sample_count);
        $display("SPI overrun         : %0d", spi_overrun);
        $display("errors              : %0d", error_count);

        if (!saw_reset_low || !saw_reset_high || !saw_wreg || !saw_null ||
            !adc_init_done || adc_init_error || sample_count < 10 ||
            spi_overrun || error_count != 0) begin
            $display("TEST FAIL");
            $finish;
        end
        else begin
            $display("TEST PASS");
            $finish;
        end
    end

endmodule

`default_nettype wire
