`timescale 1ns/1ps
`default_nettype none
// IMPORTANT: this is a SystemVerilog testbench. Compile this file as .sv.
`define CORE dut.u_daq_core
/* Pin-level ADS131M04 -> actual daq_board_top/core -> MCU regression.
 * Inputs use only ADC/MCU pins and physical reset. Internal hierarchy is
 * READ ONLY, for scoreboards, progress triggers and IRQ observation.
 * No force/release, bypassed ADC samples, direct register writes or RMS mocks.
 * FFT is an input-checked ideal-spectrum REPLAY model, not a vendor model.
 * CASE: normal, bad_ack, drdy_glitch, crc_gap, fifo_overflow, reset.
 */
module tb_ads_fpga_mcu;
    localparam integer N=2048, W=6, TOTAL=N*W;
    reg clk=0;
    always #10 clk=~clk;
    reg rst_n=0;
    wire adc_drdy, adc_dout, model_dout, adc_cs_n, adc_sclk, adc_din;
    wire mcu_cs_n, mcu_sclk, mcu_mosi, mcu_miso, uart_tx;
    reg conversions_enable=0, bad_ack=0, corrupt_crc=0;
    reg drdy_override_enable=0, drdy_override_value=1;
    reg [31:0] conversion_limit=0;
    wire configured;
    wire [31:0] conversion_count, stream_frames, command_frames;
    string case_name;
    integer ignored_plusarg;
    string vector_dir, out_dir;
    string samples_path, adc_path, registers_path, fft_input_path, csv_path;
    integer path_fd;
    reg mutate_adc_bit;
    wire mutation_now = mutate_adc_bit && adc_model.frame_is_stream &&
                        stream_frames == 0 && adc_model.bit_index == 24 && !adc_cs_n;
    assign adc_dout = mutation_now ? ~model_dout : model_dout;

    daq_board_top dut(
        .clk_50m(clk),.rst_n(rst_n),
        .adc_drdy(adc_drdy),.adc_dout(adc_dout),.adc_cs_n(adc_cs_n),
        .adc_sclk(adc_sclk),.adc_din(adc_din),
        .mcu_cs_n(mcu_cs_n),.mcu_sclk(mcu_sclk),.mcu_mosi(mcu_mosi),
        .mcu_miso(mcu_miso),.uart_tx(uart_tx)
    );
    ads131m04_model adc_model(
        .rst_n(rst_n),.cs_n(adc_cs_n),.sclk(adc_sclk),.din(adc_din),
        .dout(model_dout),.drdy(adc_drdy),
        .conversions_enable(conversions_enable),.conversion_limit(conversion_limit),
        .bad_ack_enable(bad_ack),.corrupt_crc_enable(corrupt_crc),
        .drdy_override_enable(drdy_override_enable),.drdy_override_value(drdy_override_value),
        .configured(configured),.conversion_count(conversion_count),
        .stream_frames(stream_frames),.command_frames(command_frames)
    );
    mcu_spi_master_bfm mcu(
        .cs_n(mcu_cs_n),.sclk(mcu_sclk),.mosi(mcu_mosi),.miso(mcu_miso)
    );

    reg [95:0] expected_samples [0:TOTAL-1];
    reg [143:0] expected_adc_frames [0:TOTAL-1];
    reg [31:0] golden_registers [0:W*4*7-1];
    reg [127:0] expected_fft_input [0:TOTAL-1];
    integer sample_seen=0, metrics_seen=0, fft_seen=0, axis_seen=0;
    integer assertions_checked=0, readback_words=0;
    reg previous_sample_valid=0;
    integer csv_fd;
    reg [31:0] value, held_seq;
    reg [7:0] ignored_byte, a,b,c,d;
    integer widx, offset;

    function automatic [31:0] gold(input integer window_no, input integer channel_no,
                                   input integer register_offset);
        begin gold=golden_registers[((window_no-1)*4+channel_no)*7+register_offset]; end
    endfunction
    task automatic equal32(input [31:0] got, input [31:0] expected, input string label_text);
        begin
            if (got !== expected)
                $fatal(1,"CHECK_%s: actual=%08h expected=%08h time=%0t",label_text,got,expected,$time);
            assertions_checked=assertions_checked+1;
        end
    endtask
    task automatic expect_reg(input [7:0] address, input [31:0] expected, input string label_text);
        reg [31:0] got;
        begin mcu.read_reg(address,got); equal32(got,expected,label_text); end
    endtask

    // Scoreboard samples BEFORE NBA, exactly as the downstream RTL consumes
    // sample_valid/metrics_valid. Counters reset with the internal core reset.
    always @(posedge clk) begin
        if (!rst_n || !`CORE.core_rst_n) begin
            sample_seen=0; metrics_seen=0; fft_seen=0; axis_seen=0;
            previous_sample_valid=0;
        end else begin
            if (previous_sample_valid && `CORE.sample_valid)
                $fatal(1,"ADC_VALID_WIDTH: sample_valid held for more than one cycle");
            previous_sample_valid=`CORE.sample_valid;
            if (`CORE.sample_valid) begin
                if (sample_seen >= TOTAL) $fatal(1,"ADC_EXTRA_SAMPLE");
                if ({`CORE.ch3,`CORE.ch2,`CORE.ch1,`CORE.ch0} !== expected_samples[sample_seen])
                    $fatal(1,"ADC_SAMPLE_MISMATCH: index=%0d actual=%h expected=%h",
                           sample_seen,{`CORE.ch3,`CORE.ch2,`CORE.ch1,`CORE.ch0},expected_samples[sample_seen]);
                equal32({8'd0,`CORE.status_word},{8'd0,expected_adc_frames[sample_seen][143:120]},"ADC_STATUS");
                equal32({8'd0,`CORE.crc_word},
                    {8'd0,expected_adc_frames[sample_seen][23:0]} ^ (corrupt_crc ? 32'h100 : 32'd0),"ADC_CRC_RAW");
                sample_seen=sample_seen+1;
            end
            if (`CORE.fft_s_tvalid && `CORE.fft_s_tready) begin
                if (axis_seen >= TOTAL) $fatal(1,"AXIS_EXTRA_SAMPLE");
                if (`CORE.fft_s_tdata !== expected_fft_input[axis_seen])
                    $fatal(1,"AXIS_SAMPLE_ORDER: index=%0d",axis_seen);
                if (`CORE.fft_s_tlast !== (axis_seen % N == N-1))
                    $fatal(1,"AXIS_LAST_POSITION: index=%0d",axis_seen);
                axis_seen=axis_seen+1;
            end
            if (`CORE.metrics_valid) begin
                metrics_seen=metrics_seen+1;
                if (metrics_seen > W) $fatal(1,"METRICS_EXTRA_WINDOW");
                equal32({7'd0,`CORE.vpp0},gold(metrics_seen,0,0),"VPP0");
                equal32({7'd0,`CORE.vpp1},gold(metrics_seen,1,0),"VPP1");
                equal32({7'd0,`CORE.vpp2},gold(metrics_seen,2,0),"VPP2");
                equal32({7'd0,`CORE.vpp3},gold(metrics_seen,3,0),"VPP3");
                equal32({8'd0,`CORE.rms0},gold(metrics_seen,0,1),"RMS0");
                equal32({8'd0,`CORE.rms1},gold(metrics_seen,1,1),"RMS1");
                equal32({8'd0,`CORE.rms2},gold(metrics_seen,2,1),"RMS2");
                equal32({8'd0,`CORE.rms3},gold(metrics_seen,3,1),"RMS3");
                equal32({{8{`CORE.mean0[23]}},`CORE.mean0},gold(metrics_seen,0,3),"MEAN0");
                equal32({{8{`CORE.mean1[23]}},`CORE.mean1},gold(metrics_seen,1,3),"MEAN1");
                equal32({{8{`CORE.mean2[23]}},`CORE.mean2},gold(metrics_seen,2,3),"MEAN2");
                equal32({{8{`CORE.mean3[23]}},`CORE.mean3},gold(metrics_seen,3,3),"MEAN3");
            end
            if (`CORE.fft_peak_valid) begin
                fft_seen=fft_seen+1;
                if (fft_seen > W) $fatal(1,"FFT_EXTRA_RESULT");
                equal32(`CORE.freq_hz0,gold(fft_seen,0,2),"FREQ0");
                equal32(`CORE.freq_hz1,gold(fft_seen,1,2),"FREQ1");
                equal32(`CORE.freq_hz2,gold(fft_seen,2,2),"FREQ2");
                equal32(`CORE.freq_hz3,gold(fft_seen,3,2),"FREQ3");
                equal32({21'd0,`CORE.peak_bin0},gold(fft_seen,0,4),"BIN0");
                equal32({21'd0,`CORE.peak_bin1},gold(fft_seen,1,4),"BIN1");
                equal32({21'd0,`CORE.peak_bin2},gold(fft_seen,2,4),"BIN2");
                equal32({21'd0,`CORE.peak_bin3},gold(fft_seen,3,4),"BIN3");
            end
        end
    end

    task automatic wait_initialized;
        reg [31:0] status;
        begin
            wait(dut.adc_init_done_int || dut.adc_init_error_int);
            if (!dut.adc_init_done_int || dut.adc_init_error_int)
                $fatal(1,"INIT_FAILED_UNEXPECTEDLY");
            #117;
            equal32(command_frames,2,"INIT_TWO_FRAMES");
            equal32(stream_frames,0,"INIT_NOT_A_SAMPLE");
            mcu.legacy_read(8'h7f,status);
            equal32(status,32'ha5a71304,"LEGACY_SIGNATURE");
            expect_reg(8'h7f,32'ha5a71304,"EXTENDED_SIGNATURE");
            expect_reg(8'h01,32'h00020000,"ABI");
            mcu.read_reg(8'h00,status);
            if (!status[0] || status[1] || !status[6])
                $fatal(1,"STATUS_INIT_CONFIG: %h",status);
            expect_reg(8'h0a,64000,"FS_HZ");
            expect_reg(8'h0b,N,"WINDOW_N");
            expect_reg(8'h04,0,"INITIAL_ERRORS");
        end
    endtask
    // Poll through MCU pins, NOT a peek at live sequence registers.
    task automatic wait_sequences(input integer wanted);
        integer polls;
        reg [31:0] ms, fs;
        begin
            ms=0; fs=0; polls=0;
            while (ms < wanted || fs < wanted) begin
                mcu.read_reg(8'h50,ms); mcu.read_reg(8'h51,fs);
                polls=polls+1;
                if (polls > 3000) $fatal(1,"SEQUENCE_TIMEOUT: wanted=%0d metric=%0d fft=%0d",wanted,ms,fs);
            end
            equal32(ms,wanted,"LIVE_METRICS_SEQ");
            equal32(fs,wanted,"LIVE_FFT_SEQ");
        end
    endtask
    task automatic check_channel(input integer window_no, input integer channel_no);
        integer k;
        reg [7:0] ignored;
        reg [31:0] got;
        reg [7:0] base;
        begin
            base=(channel_no+1)*16;
            mcu.begin_frame(); mcu.xfer_byte(8'ha0,ignored); mcu.xfer_byte(base,ignored);
            for (k=0;k<7;k=k+1) begin
                mcu.read_data_word(got);
                if (got !== gold(window_no,channel_no,k))
                    $fatal(1,"MCU_RESULT: window=%0d ch=%0d offset=%0d actual=%h expected=%h",
                           window_no,channel_no,k,got,gold(window_no,channel_no,k));
                $fdisplay(csv_fd,"%0t,%0d,%0d,%02h,%08h,%08h",$time,window_no,channel_no,base+k,got,gold(window_no,channel_no,k));
                readback_words=readback_words+1;
            end
            mcu.end_frame();
        end
    endtask
    task automatic check_all_channels(input integer window_no);
        integer channel_no;
        begin for(channel_no=0;channel_no<4;channel_no=channel_no+1) check_channel(window_no,channel_no); end
    endtask
    task automatic capture_and_check(input integer window_no);
        begin
            mcu.write_reg(8'h02,1); mcu.write_reg(8'h03,1);
            expect_reg(8'h07,window_no,"SNAP_METRICS_SEQ");
            expect_reg(8'h08,window_no,"SNAP_FFT_SEQ");
            expect_reg(8'h52,3,"SNAP_PRESENT");
            check_all_channels(window_no);
        end
    endtask

    task automatic test_spi_writes;
        reg [7:0] ignored;
        reg [31:0] got;
        begin
            mcu.begin_frame(); mcu.xfer_byte(8'ha1,ignored); mcu.xfer_byte(8'h60,ignored);
            mcu.write_data_word(32'h81234567); mcu.write_data_word(32'hfedcba91); mcu.end_frame();
            mcu.begin_frame(); mcu.xfer_byte(8'ha0,ignored); mcu.xfer_byte(8'h60,ignored);
            mcu.read_data_word(got); equal32(got,32'h81234567,"BURST_SCRATCH0");
            mcu.read_data_word(got); equal32(got,32'hfedcba91,"BURST_SCRATCH1");
            mcu.end_frame();
            // Speculative prefetch of undefined 0x62 must NOT be a read error.
            expect_reg(8'h04,0,"NO_PREFETCH_SIDE_EFFECT");
        end
    endtask
    task automatic test_spi_errors;
        reg [7:0] ignored;
        reg [31:0] got;
        reg bit_rx;
        integer k;
        begin
            mcu.write_reg(8'h05,32'h90); // IRQ on protocol and access errors.
            // 31/32 bits: must not commit the incomplete word.
            mcu.begin_frame(); mcu.xfer_byte(8'ha1,ignored); mcu.xfer_byte(8'h60,ignored);
            for(k=0;k<31;k=k+1) mcu.bit_xfer(1'b0,bit_rx);
            mcu.end_frame();
            expect_reg(8'h60,32'h81234567,"PARTIAL_WORD_NOT_COMMITTED");
            expect_reg(8'h04,32'h10,"PARTIAL_WORD_ERROR");
            if (!dut.mcu_irq_unused) $fatal(1,"IRQ_NOT_ASSERTED");
            expect_reg(8'h0f,1,"SPI_ERROR_COUNTER_1");
            mcu.write_reg(8'h04,32'h10);
            expect_reg(8'h04,0,"SPI_W1C");
            if (dut.mcu_irq_unused) $fatal(1,"IRQ_NOT_CLEARED");
            // A burst is per-word commit: first complete word survives abort.
            mcu.begin_frame(); mcu.xfer_byte(8'ha1,ignored); mcu.xfer_byte(8'h60,ignored);
            mcu.write_data_word(32'h12345681); mcu.xfer_byte(8'h00,ignored); mcu.end_frame();
            expect_reg(8'h60,32'h12345681,"BURST_FIRST_WORD_COMMITTED");
            expect_reg(8'h61,32'hfedcba91,"BURST_PARTIAL_SECOND_UNCHANGED");
            // Unknown command, missing address, and RO/undefined accesses.
            mcu.begin_frame(); mcu.xfer_byte(8'he3,ignored); mcu.end_frame();
            mcu.begin_frame(); mcu.xfer_byte(8'ha1,ignored); mcu.end_frame();
            mcu.write_reg(8'h7f,0);
            expect_reg(8'h7f,32'ha5a71304,"RO_WRITE_IGNORED");
            expect_reg(8'h70,32'hdeadbaad,"UNMAPPED_READ");
            expect_reg(8'h04,32'h90,"PROTOCOL_ACCESS_ERRORS");
            expect_reg(8'h0f,4,"SPI_ERROR_COUNTER_4");
            mcu.write_reg(8'h04,32'h10);
            expect_reg(8'h04,32'h80,"SELECTIVE_W1C");
            if (!dut.mcu_irq_unused) $fatal(1,"IRQ_ACCESS_BIT_LOST");
            mcu.write_reg(8'h04,32'h80); expect_reg(8'h04,0,"ALL_ERRORS_CLEARED");
            if (dut.mcu_irq_unused) $fatal(1,"IRQ_STILL_ASSERTED");
            mcu.write_reg(8'h59,1); expect_reg(8'h0f,0,"COUNTER_CLEAR");
            mcu.write_reg(8'h05,0);
        end
    endtask

    task automatic run_normal;
        begin
            wait_initialized(); test_spi_writes();
            conversion_limit=TOTAL; conversions_enable=1;
            wait_sequences(1); capture_and_check(1);
            // One CS stays selected across a WHOLE new result publication.
            // Begin VPP0, stop SCLK halfway through its word, then read the
            // remaining CH0 record: ALL seven words must remain window 1.
            mcu.write_reg(8'h02,0);
            mcu.begin_frame(); mcu.xfer_byte(8'ha0,ignored_byte); mcu.xfer_byte(8'h10,ignored_byte);
            mcu.xfer_byte(0,a); mcu.xfer_byte(0,b);
            wait(metrics_seen >= 2 && fft_seen >= 2); #317;
            mcu.xfer_byte(0,c); mcu.xfer_byte(0,d);
            equal32({a,b,c,d},gold(1,0,0),"MID_WORD_CS_SNAPSHOT");
            for(offset=1;offset<7;offset=offset+1) begin
                mcu.read_data_word(value); equal32(value,gold(1,0,offset),"SAME_CS_NO_TEAR");
            end
            mcu.end_frame();
            expect_reg(8'h07,2,"NEXT_CS_REFRESH");
            capture_and_check(2); mcu.read_reg(8'h06,held_seq);
            // Multiple independent CS transactions must preserve window 2
            // even after live processing reaches window 3.
            wait_sequences(3);
            expect_reg(8'h06,held_seq,"CROSS_CS_SEQUENCE_HELD");
            expect_reg(8'h07,2,"CROSS_CS_METRICS_HELD");
            expect_reg(8'h08,2,"CROSS_CS_FFT_HELD");
            check_all_channels(2);
            mcu.read_reg(8'h00,value);
            if ((value & 32'h3010) !== 32'h3010) $fatal(1,"SNAP_STALE_FLAGS: %h",value);
            mcu.write_reg(8'h02,0);
            expect_reg(8'h07,3,"HOLD_RELEASE_REFRESH");
            capture_and_check(3);
            for(widx=4;widx<=W;widx=widx+1) begin
                wait_sequences(widx); capture_and_check(widx);
            end
            conversions_enable=0; #1000;
            equal32(sample_seen,TOTAL,"ADC_TOTAL");
            equal32(stream_frames,TOTAL,"ADC_FRAME_TOTAL");
            equal32(conversion_count,TOTAL,"CONVERSION_TOTAL");
            equal32(axis_seen,TOTAL,"FFT_INPUT_TOTAL");
            equal32(metrics_seen,W,"METRIC_WINDOW_TOTAL");
            equal32(fft_seen,W,"FFT_WINDOW_TOTAL");
            mcu.write_reg(8'h03,1);
            expect_reg(8'h09,TOTAL,"SNAP_SAMPLE_COUNT");
            expect_reg(8'h0c,0,"FIFO_DRAINED");
            expect_reg(8'h0d,0,"NO_FFT_DISCARDS");
            expect_reg(8'h0e,0,"NO_METRIC_DROPS");
            expect_reg(8'h04,0,"HEALTHY_CHAIN_ERRORS");
            test_spi_errors();
        end
    endtask
    task automatic run_bad_ack;
        begin
            wait(dut.adc_init_error_int); #101;
            if (dut.adc_init_done_int) $fatal(1,"BAD_ACK_ACCEPTED");
            mcu.read_reg(8'h00,value);
            if (value[1:0] !== 2'b10) $fatal(1,"BAD_ACK_STATUS: %h",value);
            expect_reg(8'h04,32'h100,"INIT_ERROR_REPORT");
            mcu.write_reg(8'h04,32'h100);
            expect_reg(8'h04,32'h100,"ACTIVE_INIT_FAULT_SET_DOMINANT");
            equal32(command_frames,2,"BAD_ACK_COMMAND_FRAMES");
            equal32(sample_seen,0,"NO_STREAM_AFTER_BAD_ACK");
        end
    endtask
    task automatic run_drdy_glitch;
        begin
            wait_initialized(); conversion_limit=1; conversions_enable=1;
            @(negedge adc_cs_n); #1000;
            drdy_override_enable=1; drdy_override_value=1; #1000;
            drdy_override_value=0; #1000; drdy_override_enable=0;
            wait(sample_seen == 1); #101;
            expect_reg(8'h04,32'h1,"DRDY_BUSY_OVERRUN");
            mcu.write_reg(8'h05,1);
            if (!dut.mcu_irq_unused) $fatal(1,"ADC_OVERRUN_IRQ");
            mcu.write_reg(8'h04,1);
            expect_reg(8'h04,1,"STICKY_SOURCE_REASSERTS_AFTER_W1C");
            equal32(stream_frames,1,"GLITCH_DOES_NOT_CREATE_SECOND_FRAME");
        end
    endtask
    task automatic run_crc_gap;
        begin
            wait_initialized(); corrupt_crc=1; conversion_limit=1; conversions_enable=1;
            wait(sample_seen == 1); #101;
            expect_reg(8'h55,{8'd0,expected_adc_frames[0][23:0]} ^ 32'h100,"CORRUPTED_RAW_CRC_READABLE");
            expect_reg(8'h04,0,"CURRENT_RTL_DOES_NOT_CHECK_CRC");
            equal32(sample_seen,1,"BAD_CRC_SAMPLE_STILL_ACCEPTED");
            $display("KNOWN_GAP: bad output CRC is accepted by current RTL; this is NOT CRC protection");
        end
    endtask
    task automatic run_fifo_overflow;
        begin
            wait_initialized(); conversion_limit=2*N+128; conversions_enable=1;
            wait(sample_seen == 2*N+128); #1000;
            equal32(metrics_seen,2,"METRICS_CONTINUE_WITH_FFT_BLOCKED");
            equal32(axis_seen,0,"FFT_STALL_FOREVER");
            equal32(fft_seen,0,"NO_FALSE_FFT_PUBLISH");
            expect_reg(8'h0d,128,"FFT_DROPPED_128");
            expect_reg(8'h04,2,"FFT_DISCARD_ERROR");
            expect_reg(8'h0c,4096,"FIFO_FULL_COUNT");
            mcu.read_reg(8'h00,value);
            if (!value[9]) $fatal(1,"FFT_PATH_FAULT_NOT_LATCHED");
            mcu.write_reg(8'h04,2); expect_reg(8'h04,0,"REPORT_CLEAR_WITH_NO_NEW_EVENT");
            mcu.read_reg(8'h00,value);
            if (!value[9]) $fatal(1,"W1C_MUST_NOT_REPAIR_FFT_PATH");
            conversion_limit=2*N+129; wait(sample_seen == 2*N+129); #101;
            expect_reg(8'h0d,129,"FAULTED_PATH_CONTINUES_DISCARDING");
            expect_reg(8'h04,2,"NEW_DISCARD_REASSERTS_ERROR");
        end
    endtask
    task automatic run_reset;
        begin
            wait_initialized(); conversion_limit=64; conversions_enable=1;
            wait(sample_seen >= 16);
            mcu.begin_frame(); mcu.xfer_byte(8'ha1,ignored_byte); mcu.xfer_byte(8'h60,ignored_byte);
            mcu.xfer_byte(8'hde,ignored_byte); mcu.xfer_byte(8'had,ignored_byte);
            wait(!adc_cs_n); #113;
            conversions_enable=0; rst_n=0; #1;
            if (mcu_miso !== 1'bz) $fatal(1,"RESET_MISO_RELEASE");
            mcu.idle(); #1000; rst_n=1;
            wait_initialized();
            expect_reg(8'h60,0,"RESET_DISCARDS_PARTIAL_WRITE");
            expect_reg(8'h50,0,"RESET_METRICS_VERSION");
            expect_reg(8'h51,0,"RESET_FFT_VERSION");
            expect_reg(8'h52,0,"RESET_SNAPSHOT_EMPTY");
            equal32(sample_seen,0,"RESET_SAMPLE_COUNTER");
            conversion_limit=N; conversions_enable=1;
            wait_sequences(1); capture_and_check(1);
            equal32(sample_seen,N,"RESET_NEW_FULL_WINDOW");
            equal32(axis_seen,N,"RESET_NEW_FULL_FFT_FRAME");
            expect_reg(8'h04,0,"RESET_RECOVERED_ERRORS");
        end
    endtask

    initial begin
        case_name="normal";
        vector_dir="D:/FPGA_bag/Project/four_channels_adc/sim_vectors";
        out_dir="D:/FPGA_bag/Project/four_channels_adc";
        mutate_adc_bit=$test$plusargs("MUTATE_ADC_BIT");
        ignored_plusarg=$value$plusargs("CASE=%s",case_name);
        ignored_plusarg=$value$plusargs("E2E_VECTOR_DIR=%s",vector_dir);
        ignored_plusarg=$value$plusargs("E2E_OUT_DIR=%s",out_dir);
        if (case_name != "normal" && case_name != "bad_ack" && case_name != "drdy_glitch" &&
            case_name != "crc_gap" && case_name != "fifo_overflow" && case_name != "reset")
            $fatal(1,"UNKNOWN_CASE: %s",case_name);

        samples_path   = {vector_dir, "/samples_4ch.hex"};
        adc_path       = {vector_dir, "/adc_stream.hex"};
        registers_path = {vector_dir, "/registers.hex"};
        fft_input_path = {vector_dir, "/fft_input.hex"};
        csv_path       = {out_dir, "/mcu_readback_", case_name, ".csv"};

        path_fd=$fopen(samples_path,"r");
        if (!path_fd) $fatal(1,"E2E_VECTOR_OPEN: %s",samples_path);
        $fclose(path_fd);
        path_fd=$fopen(adc_path,"r");
        if (!path_fd) $fatal(1,"E2E_VECTOR_OPEN: %s",adc_path);
        $fclose(path_fd);
        path_fd=$fopen(registers_path,"r");
        if (!path_fd) $fatal(1,"E2E_VECTOR_OPEN: %s",registers_path);
        $fclose(path_fd);
        path_fd=$fopen(fft_input_path,"r");
        if (!path_fd) $fatal(1,"E2E_VECTOR_OPEN: %s",fft_input_path);
        $fclose(path_fd);

        $readmemh(samples_path,expected_samples);
        $readmemh(adc_path,expected_adc_frames);
        $readmemh(registers_path,golden_registers);
        $readmemh(fft_input_path,expected_fft_input);
        csv_fd=$fopen(csv_path,"w");
        if (!csv_fd) $fatal(1,"E2E_OUTPUT_OPEN: %s",csv_path);
        $fdisplay(csv_fd,"time_ps,window,channel,address,actual,expected");
        $display("START E2E case=%s; N=%0d; ADC period=15625ns; FFT=input-checked replay",case_name,N);
        $display("E2E vectors=%s output=%s",vector_dir,csv_path);
        if (case_name == "bad_ack") bad_ack=1;
        #1037; rst_n=1;
        if (case_name == "normal")
            run_normal();
        else if (case_name == "bad_ack")
            run_bad_ack();
        else if (case_name == "drdy_glitch")
            run_drdy_glitch();
        else if (case_name == "crc_gap")
            run_crc_gap();
        else if (case_name == "fifo_overflow")
            run_fifo_overflow();
        else if (case_name == "reset")
            run_reset();
        #1001;
        if (mcu_cs_n !== 1'b1 || mcu_miso !== 1'bz) $fatal(1,"FINAL_SPI_IDLE");
        $fclose(csv_fd);
        $display("PASS E2E case=%s samples=%0d metrics=%0d fft=%0d axis_words=%0d readback_words=%0d checks=%0d time=%0t",
                 case_name,sample_seen,metrics_seen,fft_seen,axis_seen,readback_words,assertions_checked,$time);
        $finish;
    end
    initial begin
        #500000000;
        $fatal(1,"GLOBAL_TIMEOUT: E2E did not finish within 500ms of simulated time");
    end

    // Bounded VCD by default: avoid recording millions of unused clock edges.
    integer wave_start_ns, wave_length_ns, wave_plusarg;
    initial begin
        if ($test$plusargs("WAVES")) begin
            wave_start_ns=2000000; wave_length_ns=1000000;
            wave_plusarg=$value$plusargs("WAVE_START_NS=%d",wave_start_ns);
            wave_plusarg=$value$plusargs("WAVE_LENGTH_NS=%d",wave_length_ns);
            $dumpfile("build/e2e/chain.vcd");
            $dumpvars(1,tb_ads_fpga_mcu);
            $dumpvars(1,dut);
            $dumpvars(1,`CORE);
            $dumpvars(1,`CORE.u_mcu_spi);
            $dumpoff;
            #(wave_start_ns); $dumpon;
            #(wave_length_ns); $dumpoff;
        end
    end
endmodule
`undef CORE
`default_nettype wire
