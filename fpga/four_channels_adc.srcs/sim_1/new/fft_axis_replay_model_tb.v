`timescale 1ns/1ps
`default_nettype none
/* LOCAL IMPORT FIX -- simulation only.
 * No include file or global E2E_* macro is required by this source.
 * Keep the .v filename, but set Vivado File Type to SystemVerilog ($fatal).
 * Default vector location matches the user's project; change VECTOR_DIR or
 * supply +E2E_VECTOR_DIR=<directory> when using another location.
 * The canonical module preserves the original E2E hierarchy. A *_tb alias
 * is provided at the end for the module name pasted from the local project.
 * These are driven MODELS, not self-running simulation top modules.
 */
/* SIMULATION ONLY. Input-checked FFT AXI response/replay model.
 * Checks EVERY accepted input against independently generated stimulus,
 * configuration and TLAST, then returns that window's Python FFT spectrum.
 * No spectrum is emitted before all 2048 input words have been verified.
 * Therefore this verifies FIFO/feeder + peak finder + MCU data transport,
 * NOT the numerical implementation, scaling/rounding or timing of Xilinx FFT.
 * Default controlled input stalls/output gaps exercise AXI flow control.
 * Use DAQ_FFT_MODULE=daq_fft_replay_model and DAQ_FFT_HAS_ARESETN for E2E only.
 */
module daq_fft_replay_model #(
    parameter integer NUM_WINDOWS = 6,
    parameter VECTOR_DIR = "D:/FPGA_bag/Project/four_channels_adc/sim_vectors"
)(
    input wire aclk,
    input wire aresetn,
    input wire [103:0] s_axis_config_tdata,
    input wire s_axis_config_tvalid,
    output wire s_axis_config_tready,
    input wire [127:0] s_axis_data_tdata,
    input wire s_axis_data_tvalid,
    output wire s_axis_data_tready,
    input wire s_axis_data_tlast,
    output reg [127:0] m_axis_data_tdata,
    output reg [15:0] m_axis_data_tuser,
    output reg m_axis_data_tvalid,
    input wire m_axis_data_tready,
    output reg m_axis_data_tlast,
    output reg event_frame_started,
    output reg event_tlast_unexpected,
    output reg event_tlast_missing,
    output wire event_status_channel_halt,
    output wire event_data_in_channel_halt,
    output wire event_data_out_channel_halt
);
    // The configuration below is explicitly a 2048-point configuration.
    localparam integer FFT_N = 2048;
    localparam integer NUM_SAMPLES = FFT_N * NUM_WINDOWS;
    reg [127:0] input_words [0:NUM_SAMPLES-1];
    reg [127:0] output_words [0:NUM_SAMPLES-1];
    localparam [103:0] EXPECT_CONFIG =
        {4'd0,22'h155555,22'h155555,22'h155555,22'h155555,4'hf,3'd0,5'd11};
    localparam CAPTURE=0, EMIT=1;
    integer state, cycle_count, input_index, output_index, frame_count;
    reg configured, previous_stall;
    reg [127:0] held_input;
    reg held_last;
    reg stall_forever;
    // Packed text buffers also avoid the original "string is unknown" error.
    reg [8*64-1:0] case_name;
    reg [8*1024-1:0] vector_dir;
    reg [8*1152-1:0] input_path, output_path;
    integer ignored_plusarg, vector_fd, vector_index;
    assign s_axis_config_tready = aresetn && !configured && cycle_count >= 7;
    assign s_axis_data_tready = aresetn && configured && state == CAPTURE &&
                               !stall_forever && (cycle_count % 5 != 0);
    assign event_status_channel_halt=1'b0;
    assign event_data_in_channel_halt=1'b0;
    assign event_data_out_channel_halt=1'b0;
    initial begin
        if (NUM_WINDOWS < 1 || NUM_WINDOWS > 1048575)
            $fatal(1,"FFT_MODEL_PARAMETER: NUM_WINDOWS out of supported range");
        vector_dir=VECTOR_DIR;
        ignored_plusarg=$value$plusargs("E2E_VECTOR_DIR=%s",vector_dir);
        if (vector_dir == 0) $fatal(1,"FFT_VECTOR_DIR: empty directory");
        $sformat(input_path,"%0s/fft_input.hex",vector_dir);
        $sformat(output_path,"%0s/fft_output.hex",vector_dir);

        vector_fd=$fopen(input_path,"r");
        if (vector_fd == 0)
            $fatal(1,"FFT_VECTOR_OPEN: cannot open %0s",input_path);
        $fclose(vector_fd);
        vector_fd=$fopen(output_path,"r");
        if (vector_fd == 0)
            $fatal(1,"FFT_VECTOR_OPEN: cannot open %0s",output_path);
        $fclose(vector_fd);

        for (vector_index=0; vector_index<NUM_SAMPLES; vector_index=vector_index+1) begin
            input_words[vector_index]={128{1'bx}};
            output_words[vector_index]={128{1'bx}};
        end
        $readmemh(input_path,input_words,0,NUM_SAMPLES-1);
        $readmemh(output_path,output_words,0,NUM_SAMPLES-1);
        // Reject unread/short/X/Z vectors, rather than simulating with garbage.
        for (vector_index=0; vector_index<NUM_SAMPLES; vector_index=vector_index+1) begin
            if ((^input_words[vector_index]) === 1'bx)
                $fatal(1,"FFT_VECTOR_DATA: input word %0d is missing or contains X/Z",vector_index);
            if ((^output_words[vector_index]) === 1'bx)
                $fatal(1,"FFT_VECTOR_DATA: output word %0d is missing or contains X/Z",vector_index);
        end
        $display("FFT_MODEL_VECTORS_OK: count=%0d directory=%0s",NUM_SAMPLES,vector_dir);
        case_name="normal";
        ignored_plusarg=$value$plusargs("CASE=%s",case_name);
        stall_forever=(case_name == "fifo_overflow");
    end
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state<=CAPTURE; cycle_count<=0; input_index<=0; output_index<=0;
            frame_count<=0; configured<=0; previous_stall<=0;
            held_input<=0; held_last<=0;
            m_axis_data_tdata<=0; m_axis_data_tuser<=0;
            m_axis_data_tvalid<=0; m_axis_data_tlast<=0;
            event_frame_started<=0; event_tlast_unexpected<=0; event_tlast_missing<=0;
        end else begin
            cycle_count<=cycle_count+1;
            event_frame_started<=0; event_tlast_unexpected<=0; event_tlast_missing<=0;
            if (s_axis_config_tvalid && s_axis_config_tready) begin
                if (s_axis_config_tdata !== EXPECT_CONFIG)
                    $fatal(1,"FFT_CONFIG: configuration word mismatch: %h",s_axis_config_tdata);
                configured<=1;
            end
            if (!configured && s_axis_data_tvalid)
                $fatal(1,"FFT_EARLY_DATA: TVALID appeared before configuration");
            if (previous_stall) begin
                if (!s_axis_data_tvalid || s_axis_data_tdata !== held_input ||
                    s_axis_data_tlast !== held_last)
                    $fatal(1,"FFT_AXIS_STABILITY: data/last/valid changed while stalled");
            end
            previous_stall<=s_axis_data_tvalid && !s_axis_data_tready;
            if (s_axis_data_tvalid && !s_axis_data_tready) begin
                held_input<=s_axis_data_tdata; held_last<=s_axis_data_tlast;
            end
            if (s_axis_data_tvalid && s_axis_data_tready) begin
                if (frame_count >= NUM_WINDOWS)
                    $fatal(1,"FFT_EXTRA_FRAME");
                if (s_axis_data_tdata !== input_words[frame_count*FFT_N+input_index])
                    $fatal(1,"FFT_INPUT_MISMATCH: window=%0d sample=%0d actual=%h expected=%h",
                           frame_count+1,input_index,s_axis_data_tdata,
                           input_words[frame_count*FFT_N+input_index]);
                if (s_axis_data_tlast !== (input_index == FFT_N-1))
                    $fatal(1,"FFT_TLAST: expected TLAST only on sample 2047");
                if (input_index == 0) event_frame_started<=1;
                if (input_index == FFT_N-1) begin
                    state<=EMIT; input_index<=0; output_index<=0;
                end else input_index<=input_index+1;
            end
            // Registered valid/data: once valid, keep stable until accepted.
            if (state == EMIT) begin
                if (m_axis_data_tvalid && m_axis_data_tready) begin
                    m_axis_data_tvalid<=0;
                    if (output_index == FFT_N-1) begin
                        state<=CAPTURE; frame_count<=frame_count+1; output_index<=0;
                    end else output_index<=output_index+1;
                end else if (!m_axis_data_tvalid && cycle_count % 7 != 0) begin
                    m_axis_data_tdata<=output_words[frame_count*FFT_N+output_index];
                    m_axis_data_tuser<=output_index[15:0];
                    m_axis_data_tlast<=(output_index == FFT_N-1);
                    m_axis_data_tvalid<=1;
                end
            end
        end
    end
endmodule

// Optional compatibility alias. Prefer the canonical name above in the
// supplied E2E top: its internal observer paths (e.g. adc_model.bit_index)
// intentionally refer directly to the canonical implementation.
module daq_fft_replay_model_tb #(
    parameter integer NUM_WINDOWS = 6,
    parameter VECTOR_DIR = "D:/FPGA_bag/Project/four_channels_adc/sim_vectors"
)(
    input wire aclk,
    input wire aresetn,
    input wire [103:0] s_axis_config_tdata,
    input wire s_axis_config_tvalid,
    output wire s_axis_config_tready,
    input wire [127:0] s_axis_data_tdata,
    input wire s_axis_data_tvalid,
    output wire s_axis_data_tready,
    input wire s_axis_data_tlast,
    output wire [127:0] m_axis_data_tdata,
    output wire [15:0] m_axis_data_tuser,
    output wire m_axis_data_tvalid,
    input wire m_axis_data_tready,
    output wire m_axis_data_tlast,
    output wire event_frame_started,
    output wire event_tlast_unexpected,
    output wire event_tlast_missing,
    output wire event_status_channel_halt,
    output wire event_data_in_channel_halt,
    output wire event_data_out_channel_halt
);
    daq_fft_replay_model #(
        .NUM_WINDOWS(NUM_WINDOWS),
        .VECTOR_DIR(VECTOR_DIR)
    ) u_model (
        .aclk(aclk),
        .aresetn(aresetn),
        .s_axis_config_tdata(s_axis_config_tdata),
        .s_axis_config_tvalid(s_axis_config_tvalid),
        .s_axis_config_tready(s_axis_config_tready),
        .s_axis_data_tdata(s_axis_data_tdata),
        .s_axis_data_tvalid(s_axis_data_tvalid),
        .s_axis_data_tready(s_axis_data_tready),
        .s_axis_data_tlast(s_axis_data_tlast),
        .m_axis_data_tdata(m_axis_data_tdata),
        .m_axis_data_tuser(m_axis_data_tuser),
        .m_axis_data_tvalid(m_axis_data_tvalid),
        .m_axis_data_tready(m_axis_data_tready),
        .m_axis_data_tlast(m_axis_data_tlast),
        .event_frame_started(event_frame_started),
        .event_tlast_unexpected(event_tlast_unexpected),
        .event_tlast_missing(event_tlast_missing),
        .event_status_channel_halt(event_status_channel_halt),
        .event_data_in_channel_halt(event_data_in_channel_halt),
        .event_data_out_channel_halt(event_data_out_channel_halt)
    );
endmodule
`default_nettype wire
