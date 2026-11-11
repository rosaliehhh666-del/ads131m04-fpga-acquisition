`timescale 1ns / 1ps
`default_nettype none
/*
 * raw_fifo_4ch.v
 *
 * Decimated raw ADC data FIFO for MCU readout.
 *
 * Captures 4-channel 24-bit ADC samples, decimates by DECIMATION (default 8),
 * and stores them in a FIFO_DEPTH-deep FIFO. MCU reads one frame per 4 SPI
 * words (0x62~0x65). The FIFO pop happens when reg_rd_strobe fires on the
 * LAST word (address 0x65), after all four channels of the frame have been
 * read from an atomic snapshot.
 *
 * P0-1 fixes (2026-10-21):
 *   - Memory addressing uses ADDR_W = clog2(FIFO_DEPTH) bits only.
 *     The write/read pointers keep ADDR_W+1 bits for full/empty/count,
 *     but the array index never uses the wrap bit (previous bug: COUNT_W=13
 *     pointer used directly as a 4096-deep array index, wrapping out of range
 *     at 4096 and beyond).
 *   - FIFO_DEPTH must be a power of two; verified at elaboration.
 *   - DECIMATION=1 no longer produces a zero/negative-width counter.
 *   - Four-channel readout uses an atomic snapshot: reading 0x62 latches all
 *     four channels of the frame at the current read pointer. Words 0x63/0x64
 *     return snapshot words, and 0x65 returns the last word and pops the frame
 *     (read pointer advances) only if the snapshot is valid. Reads of an empty
 *     FIFO consume nothing, so an interrupted transaction can be re-read.
 *   - raw_frame_number increments on every successful pop; a read of an empty
 *     FIFO or an out-of-order partial read does NOT consume a frame.
 *
 * Output data is sign-extended from 24-bit to 32-bit.
 *
 * Register interface:
 *   0x62 raw_ch0  (read: frame start; latches atomic snapshot, 32-bit signed)
 *   0x63 raw_ch1  (read: snapshot CH1)
 *   0x64 raw_ch2  (read: snapshot CH2)
 *   0x65 raw_ch3  (read: snapshot CH3 + FIFO pop for this frame)
 *   0x66 raw_fifo_level
 *   0x67 raw_dropped
 *   0x68 raw_decimation
 *   0x69 raw_frame_number
 *
 * BRAM inference: memory arrays use synchronous write with no reset,
 * matching Vivado's block RAM template.
 */
module raw_fifo_4ch #(
    parameter integer DECIMATION = 8,
    parameter integer FIFO_DEPTH = 4096,
    parameter integer COUNT_W    = 13
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_valid,
    input  wire signed [23:0] ch0,
    input  wire signed [23:0] ch1,
    input  wire signed [23:0] ch2,
    input  wire signed [23:0] ch3,
    input  wire        reg_rd_strobe,
    input  wire [7:0]  reg_rd_addr,
    output reg  [31:0] raw_ch0,
    output reg  [31:0] raw_ch1,
    output reg  [31:0] raw_ch2,
    output reg  [31:0] raw_ch3,
    output reg  [31:0] raw_fifo_level,
    output reg  [31:0] raw_dropped,
    output wire [31:0] raw_decimation,
    output reg  [31:0] raw_frame_number
);

    assign raw_decimation = DECIMATION[31:0];

    // ------------------------------------------------------------------
    // Parameter checks (elaboration-time only)
    // ------------------------------------------------------------------
    // synthesis translate_off
    initial begin
        if (FIFO_DEPTH < 2 || (FIFO_DEPTH & (FIFO_DEPTH-1)) != 0)
            $fatal(1, "raw_fifo_4ch: FIFO_DEPTH must be a power of two >= 2");
        if (DECIMATION < 1)
            $fatal(1, "raw_fifo_4ch: DECIMATION must be >= 1");
    end
    // synthesis translate_on

    // ------------------------------------------------------------------
    // Address geometry
    //   ADDR_W  : memory array index width (low bits only, no wrap bit)
    //   PTR_W   : ADDR_W+1 circular pointers (wrap bit used for full/empty)
    // ------------------------------------------------------------------
    localparam integer ADDR_W = $clog2(FIFO_DEPTH);
    localparam integer PTR_W  = ADDR_W + 1;

    // Decimation counter — max(1, clog2(DECIMATION)) protects DECIMATION=1
    localparam integer DECIM_W =
        (DECIMATION <= 1) ? 1 : $clog2(DECIMATION);

    reg [DECIM_W-1:0] decim_cnt;
    wire decim_tick = sample_valid && (decim_cnt == DECIMATION[DECIM_W-1:0]-1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            decim_cnt <= {DECIM_W{1'b0}};
        else if (sample_valid)
            decim_cnt <= (decim_cnt == DECIMATION[DECIM_W-1:0]-1'b1) ?
                         {DECIM_W{1'b0}} : decim_cnt + 1'b1;
    end

    // Sign-extend 24-bit to 32-bit
    wire [31:0] ch0_ext = {{8{ch0[23]}}, ch0};
    wire [31:0] ch1_ext = {{8{ch1[23]}}, ch1};
    wire [31:0] ch2_ext = {{8{ch2[23]}}, ch2};
    wire [31:0] ch3_ext = {{8{ch3[23]}}, ch3};

    // Block RAM arrays — NO async reset on the memory itself
    (* ram_style = "block" *) reg [31:0] mem_ch0 [0:FIFO_DEPTH-1];
    (* ram_style = "block" *) reg [31:0] mem_ch1 [0:FIFO_DEPTH-1];
    (* ram_style = "block" *) reg [31:0] mem_ch2 [0:FIFO_DEPTH-1];
    (* ram_style = "block" *) reg [31:0] mem_ch3 [0:FIFO_DEPTH-1];

    // Circular pointers with wrap bit; count = wr_ptr - rd_ptr (PTR_W bits)
    reg [PTR_W-1:0] wr_ptr;
    reg [PTR_W-1:0] rd_ptr;

    wire [PTR_W-1:0] fifo_count = wr_ptr - rd_ptr;
    wire fifo_full  = (fifo_count[ADDR_W] == 1'b1) &&
                      (fifo_count[ADDR_W-1:0] == {ADDR_W{1'b0}});
    wire fifo_empty = (fifo_count == {PTR_W{1'b0}});

    reg [31:0] dropped_count;

    // Frame read state machine:
    //   0 = idle / frame consumed
    //   1 = snapshot latched, CH0 word read
    //   2 = CH1 word read
    //   3 = CH2 word read
    //   4 = CH3 word read (frame complete, pop issued)
    reg [2:0] read_pos;
    reg       snapshot_valid;
    reg [31:0] snap0, snap1, snap2, snap3;

    wire is_read_0x62 = reg_rd_strobe && (reg_rd_addr == 8'h62);
    wire is_read_0x63 = reg_rd_strobe && (reg_rd_addr == 8'h63);
    wire is_read_0x64 = reg_rd_strobe && (reg_rd_addr == 8'h64);
    wire is_read_0x65 = reg_rd_strobe && (reg_rd_addr == 8'h65);

    // Snapshot latches on the 0x62 read (frame start), only when non-empty.
    // Memory is a synchronous-read BRAM: address is presented this cycle and
    // data appears next cycle, so the snapshot uses the previously registered
    // read data. To keep the read pointer stable during a frame, rd_ptr is
    // only advanced by an explicit pop (0x65), never by a 0x62 latch.
    wire [ADDR_W-1:0] rd_addr = rd_ptr[ADDR_W-1:0];

    // -------------------------------------------------------
    // BRAM write: synchronous, no reset on the array
    // -------------------------------------------------------
    always @(posedge clk) begin
        if (decim_tick && !fifo_full) begin
            mem_ch0[wr_ptr[ADDR_W-1:0]] <= ch0_ext;
            mem_ch1[wr_ptr[ADDR_W-1:0]] <= ch1_ext;
            mem_ch2[wr_ptr[ADDR_W-1:0]] <= ch2_ext;
            mem_ch3[wr_ptr[ADDR_W-1:0]] <= ch3_ext;
        end
    end

    // -------------------------------------------------------
    // BRAM read: registered output (1 cycle latency)
    // Continuously follows rd_ptr so 0x62 can capture it on demand.
    // -------------------------------------------------------
    reg [31:0] mem_r0, mem_r1, mem_r2, mem_r3;
    always @(posedge clk) begin
        mem_r0 <= mem_ch0[rd_addr];
        mem_r1 <= mem_ch1[rd_addr];
        mem_r2 <= mem_ch2[rd_addr];
        mem_r3 <= mem_ch3[rd_addr];
    end

    // -------------------------------------------------------
    // Control logic (async reset OK here — no BRAM in this block)
    // -------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr          <= {PTR_W{1'b0}};
            rd_ptr          <= {PTR_W{1'b0}};
            dropped_count   <= 32'd0;
            raw_fifo_level  <= 32'd0;
            raw_dropped     <= 32'd0;
            raw_frame_number<= 32'd0;
            read_pos        <= 3'd0;
            snapshot_valid  <= 1'b0;
            snap0           <= 32'd0;
            snap1           <= 32'd0;
            snap2           <= 32'd0;
            snap3           <= 32'd0;
            raw_ch0         <= 32'd0;
            raw_ch1         <= 32'd0;
            raw_ch2         <= 32'd0;
            raw_ch3         <= 32'd0;
        end else begin
            // --------------------------------------------------
            // Write pointer / drop counter
            // --------------------------------------------------
            if (decim_tick && !fifo_full)
                wr_ptr <= wr_ptr + 1'b1;

            if (decim_tick && fifo_full)
                dropped_count <= dropped_count + 1'b1;

            // --------------------------------------------------
            // Frame read: atomic snapshot + pop on last word
            // --------------------------------------------------
            case (read_pos)
                3'd0: begin
                    // Frame start: 0x62 latches snapshot (non-empty only).
                    if (is_read_0x62 && !fifo_empty) begin
                        snap0        <= mem_r0;
                        snap1        <= mem_r1;
                        snap2        <= mem_r2;
                        snap3        <= mem_r3;
                        snapshot_valid <= 1'b1;
                        read_pos     <= 3'd1;
                        raw_ch0      <= mem_r0;
                    end else if (is_read_0x62) begin
                        // Empty read: nothing consumed, keep 0 output.
                        raw_ch0      <= 32'd0;
                    end
                end
                3'd1: begin
                    if (is_read_0x62) begin
                        // Interrupted mid-frame: re-latch the same frame
                        // (rd_ptr has not advanced), restart at CH0.
                        snap0        <= mem_r0;
                        snap1        <= mem_r1;
                        snap2        <= mem_r2;
                        snap3        <= mem_r3;
                        raw_ch0      <= mem_r0;
                        read_pos     <= 3'd1;
                    end else if (is_read_0x63) begin
                        raw_ch1  <= snap1;
                        read_pos <= 3'd2;
                    end
                end
                3'd2: begin
                    if (is_read_0x62) begin
                        snap0        <= mem_r0;
                        snap1        <= mem_r1;
                        snap2        <= mem_r2;
                        snap3        <= mem_r3;
                        raw_ch0      <= mem_r0;
                        read_pos     <= 3'd1;
                    end else if (is_read_0x64) begin
                        raw_ch2  <= snap2;
                        read_pos <= 3'd3;
                    end
                end
                3'd3: begin
                    if (is_read_0x62) begin
                        snap0        <= mem_r0;
                        snap1        <= mem_r1;
                        snap2        <= mem_r2;
                        snap3        <= mem_r3;
                        raw_ch0      <= mem_r0;
                        read_pos     <= 3'd1;
                    end else if (is_read_0x65) begin
                        raw_ch3   <= snap3;
                        read_pos  <= 3'd4;
                    end
                end
                3'd4: begin
                    // Pop issued by the previous 0x65 read.
                    read_pos       <= 3'd0;
                    snapshot_valid <= 1'b0;
                    rd_ptr         <= rd_ptr + 1'b1;
                    raw_frame_number <= raw_frame_number + 1'b1;
                end
                default: read_pos <= 3'd0;
            endcase

            // --------------------------------------------------
            // Status outputs (level/dropped update every cycle)
            // --------------------------------------------------
            raw_fifo_level <= {19'd0, fifo_count[PTR_W-1:0]};
            raw_dropped    <= dropped_count;
        end
    end

endmodule
`default_nettype wire
