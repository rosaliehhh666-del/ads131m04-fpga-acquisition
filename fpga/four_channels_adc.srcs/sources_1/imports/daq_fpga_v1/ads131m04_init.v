`timescale 1ns / 1ps
`default_nettype none

/*
 * ads131m04_init.v
 *
 * ADS131M04 initialization — multi-phase:
 *
 *   Phase 1: RREG ID      — verify basic SPI communication (0x24xx)
 *   Phase 2: WREG CLOCK   — OSR / data rate
 *   Phase 3: WREG GAIN1/GAIN2/CFG — per-channel PGA and CFG (CRC enable etc.)
 *   Phase 4: RREG read-back verification of CLOCK, GAIN1, GAIN2, CFG
 *
 * Full flow:
 *
 *   RESET_LOW  (20ms)
 *       ↓
 *   RESET_HIGH
 *       ↓
 *   WAIT_DRDY  (first DRDY after reset)
 *       ↓
 *   SEND_ID → WAIT_ID → SEND_NULL_ID → WAIT_NULL_ID → CHECK_ID
 *       ↓
 *   SEND_WREG → WAIT_WREG → SEND_NULL_WREG → WAIT_NULL_WREG → CHECK_ACK
 *       ↓
 *   SEND_WREG_CFG → WAIT_WREG_CFG → SEND_NULL_WREG_CFG → WAIT_NULL_WREG_CFG
 *       → CHECK_ACK_CFG
 *       ↓
 *   READBACK loop over [CLOCK, GAIN1, GAIN2, CFG]:
 *       SEND_RREG → WAIT_RREG → SEND_NULL_RREG → WAIT_NULL_RREG → CHECK_RREG
 *       ↓
 *   SETTLE (80 ms for sinc filter)
 *       ↓
 *   DONE (cfg_id incremented, init_done asserted)
 *
 * P0-2 (2026-10-21):
 *   - Sampling rate comes from a single configuration source: WREG_CLK_VALUE
 *     (default 0x0F06: CLOCK[3:1]=011 -> OSR=256, fDATA = 8.192MHz/(2*256)
 *     = 16 kSPS in high-resolution mode). fs_hz is derived from the same
 *     parameters and exported so FFT scaling, status registers and the host
 *     all agree.
 *   - Every register written is read back and verified before the stream
 *     starts. Read-back failure retries the whole write phase.
 *   - cfg_id increments on every successful configuration so the host can
 *     tell that a config change was applied and flushed.
 *
 * P1-1 (2026-10-21):
 *   - Per-channel PGA via GAIN1_VALUE/GAIN2_VALUE parameters
 *     (PGA_GAINn[2:0]: 0=1x, 1=2x, ..., 7=128x) and CFG via CFG_VALUE
 *     (CRC_EN[3:0] in low nibble, DCBLOCK_EN at bit 8).
 *   - cfg_reconfig re-runs phases 2..4 (with read-back) and clears
 *     init_done during the transition, so the host stops publishing results,
 *     waits for filter settle, flushes old data and resumes with the new
 *     cfg_id.
 */

module ads131m04_init #(
    parameter integer CLK_FREQ             = 50_000_000,
    parameter integer STARTUP_DELAY_CYCLES = 1_000_000,  // 20 ms
    parameter integer SETTLE_MS            = 80,          // 80 ms
    parameter integer MAX_ID_RETRIES       = 3,
    parameter integer MAX_WREG_RETRIES     = 3,
    parameter integer WREG_CLK_VALUE       = 16'h0F06,    // OSR=256 -> 16 kSPS
    parameter integer FS_HZ                = 16000,       // fDATA in Hz
    parameter integer GAIN1_VALUE          = 16'h0000,    // CH1 PGA=1x, CH2 PGA=1x
    parameter integer GAIN2_VALUE          = 16'h0000,    // CH3 PGA=1x, CH4 PGA=1x
    parameter integer CFG_VALUE            = 16'h0000     // CRC disabled by default
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         spi_busy,
    input  wire         cmd_done,
    input  wire [143:0] cmd_rx_frame,

    input  wire         drdy_event,

    input  wire         cfg_reconfig,       // 1-cycle pulse: re-run phases 2..4

    output reg          cmd_start,
    output reg  [143:0] cmd_tx_frame,

    output reg          stream_enable,
    output reg          adc_reset_n,
    output reg          init_done,
    output reg          init_error,

    output reg  [3:0]   retry_count,
    output reg  [4:0]   init_state,
    output reg  [23:0]  wreg_ack_word,

    // P0-2 / P1-1 metadata
    output reg  [31:0]  cfg_id,             // configuration generation number
    output reg  [31:0]  fs_hz,              // actual configured sample rate
    output reg  [15:0]  gain1_readback,
    output reg  [15:0]  gain2_readback,
    output reg  [15:0]  cfg_readback,

    // Additional debug outputs
    output reg  [23:0]  id_word,
    output reg  [23:0]  debug_tx_word0
);

    // ============================================================
    // State definitions (5-bit, 0-31)
    // ============================================================
    localparam [4:0]
        ST_RESET_LOW       = 5'h00,
        ST_RESET_HIGH      = 5'h01,
        ST_WAIT_DRDY       = 5'h02,
        ST_SEND_ID         = 5'h03,
        ST_WAIT_ID         = 5'h04,
        ST_SEND_NULL_ID    = 5'h05,
        ST_WAIT_NULL_ID    = 5'h06,
        ST_CHECK_ID        = 5'h07,
        ST_SEND_WREG       = 5'h08,
        ST_WAIT_WREG       = 5'h09,
        ST_SEND_NULL_WREG  = 5'h0A,
        ST_WAIT_NULL_WREG  = 5'h0B,
        ST_CHECK_ACK       = 5'h0C,
        ST_SEND_WREG_CFG   = 5'h0D,
        ST_WAIT_WREG_CFG   = 5'h0E,
        ST_SEND_NULL_WREG_CFG = 5'h0F,
        ST_WAIT_NULL_WREG_CFG = 5'h10,
        ST_CHECK_ACK_CFG   = 5'h11,
        ST_RREG_SEND       = 5'h12,
        ST_WAIT_RREG       = 5'h13,
        ST_SEND_NULL_RREG  = 5'h14,
        ST_WAIT_NULL_RREG  = 5'h15,
        ST_CHECK_RREG      = 5'h16,
        ST_SETTLE          = 5'h17,
        ST_DONE            = 5'h18,
        ST_ERROR           = 5'h19;

    // ============================================================
    // Command constants
    // ============================================================
    // RREG ID: addr=0x00, count=1  →  1010_00000_00000000_0 = 0xA00000
    localparam [23:0] RREG_ID_CMD      = 24'hA00000;
    // WREG CLOCK: addr=0x03, count=1 →  0110_00011_00000000_0 = 0x618000
    localparam [23:0] WREG_CLOCK_CMD   = 24'h618000;
    localparam [23:0] WREG_CLOCK_DATA  = {WREG_CLK_VALUE, 8'h00};
    // WREG GAIN1: addr=0x04, count=2 (GAIN1, GAIN2, CFG)
    // → 0110_00100_0_00010_0 = 0x620800
    localparam [23:0] WREG_CFG_CMD     = 24'h620800;
    // RREG commands for read-back (addr in bits [19:15])
    localparam [23:0] RREG_CLOCK_CMD   = 24'hA30000;   // addr 0x03
    localparam [23:0] RREG_GAIN1_CMD   = 24'hA40000;   // addr 0x04
    localparam [23:0] RREG_GAIN2_CMD   = 24'hA50000;   // addr 0x05
    localparam [23:0] RREG_CFG_CMD     = 24'hA60000;   // addr 0x06

    // Read-back verification table:
    //   index 0: CLOCK  expect WREG_CLK_VALUE
    //   index 1: GAIN1  expect GAIN1_VALUE
    //   index 2: GAIN2  expect GAIN2_VALUE
    //   index 3: CFG    expect CFG_VALUE
    reg [1:0] rb_idx;
    reg [23:0] rb_cmd;
    reg [15:0] rb_expect;

    // ============================================================
    // Counters
    // ============================================================
    localparam integer SETTLE_CYCLES = (CLK_FREQ / 1000) * SETTLE_MS;
    localparam integer CNTR_W = $clog2(STARTUP_DELAY_CYCLES > SETTLE_CYCLES ?
                                        STARTUP_DELAY_CYCLES + 1 : SETTLE_CYCLES + 1);

    reg [CNTR_W-1:0] cnt;
    reg [3:0]        id_retry_cnt;
    reg [3:0]        wreg_retry_cnt;

    // ============================================================
    // FSM
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            init_state       <= ST_RESET_LOW;
            cmd_start        <= 1'b0;
            cmd_tx_frame     <= 144'd0;
            stream_enable    <= 1'b0;
            adc_reset_n      <= 1'b0;
            init_done        <= 1'b0;
            init_error       <= 1'b0;
            retry_count      <= 4'd0;
            id_retry_cnt     <= 4'd0;
            wreg_retry_cnt   <= 4'd0;
            cnt              <= {CNTR_W{1'b0}};
            wreg_ack_word    <= 24'd0;
            id_word          <= 24'd0;
            debug_tx_word0   <= 24'd0;
            cfg_id           <= 32'd0;
            fs_hz            <= 32'd0;
            gain1_readback   <= 16'd0;
            gain2_readback   <= 16'd0;
            cfg_readback     <= 16'd0;
            rb_idx           <= 2'd0;
            rb_cmd           <= 24'd0;
            rb_expect        <= 16'd0;
        end
        else begin
            cmd_start <= 1'b0;
            adc_reset_n <= (init_state == ST_RESET_LOW) ? 1'b0 : 1'b1;

            case (init_state)

                // ------------------------------------------------
                // RESET_LOW
                // ------------------------------------------------
                ST_RESET_LOW: begin
                    if (cnt >= STARTUP_DELAY_CYCLES[CNTR_W-1:0] - 1'b1) begin
                        init_state <= ST_RESET_HIGH;
                        cnt        <= {CNTR_W{1'b0}};
                    end
                    else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // ------------------------------------------------
                // RESET_HIGH
                // ------------------------------------------------
                ST_RESET_HIGH: begin
                    init_state <= ST_WAIT_DRDY;
                end

                // ------------------------------------------------
                // WAIT_DRDY — first DRDY after reset
                // ------------------------------------------------
                ST_WAIT_DRDY: begin
                    if (drdy_event) begin
                        cmd_tx_frame   <= {RREG_ID_CMD, 120'd0};
                        debug_tx_word0 <= RREG_ID_CMD;
                        init_state     <= ST_SEND_ID;
                    end
                end

                // ================================================
                // Phase 1: RREG ID
                // ================================================

                ST_SEND_ID: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_ID;
                    end
                end

                ST_WAIT_ID: begin
                    if (cmd_done) begin
                        cmd_tx_frame   <= 144'd0;
                        debug_tx_word0 <= 24'd0;
                        init_state     <= ST_SEND_NULL_ID;
                    end
                end

                ST_SEND_NULL_ID: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_NULL_ID;
                    end
                end

                ST_WAIT_NULL_ID: begin
                    if (cmd_done) begin
                        id_word    <= cmd_rx_frame[143:120];
                        init_state <= ST_CHECK_ID;
                    end
                end

                ST_CHECK_ID: begin
                    // ID register top byte should be 0x24 (ADS131M04 family)
                    if (id_word[23:16] == 8'h24) begin
                        // ID OK → Phase 2
                        cmd_tx_frame   <= {WREG_CLOCK_CMD, WREG_CLOCK_DATA, 96'd0};
                        debug_tx_word0 <= WREG_CLOCK_CMD;
                        init_state     <= ST_SEND_WREG;
                    end
                    else begin
                        // ID failed → retry
                        if (id_retry_cnt < MAX_ID_RETRIES[3:0]) begin
                            id_retry_cnt <= id_retry_cnt + 1'b1;
                            retry_count  <= retry_count + 1'b1;
                            cmd_tx_frame   <= {RREG_ID_CMD, 120'd0};
                            debug_tx_word0 <= RREG_ID_CMD;
                            init_state     <= ST_SEND_ID;
                        end
                        else begin
                            init_state <= ST_ERROR;
                            init_error <= 1'b1;
                        end
                    end
                end

                // ================================================
                // Phase 2: WREG CLOCK
                // ================================================

                ST_SEND_WREG: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_WREG;
                    end
                end

                ST_WAIT_WREG: begin
                    if (cmd_done) begin
                        cmd_tx_frame   <= 144'd0;
                        debug_tx_word0 <= 24'd0;
                        init_state     <= ST_SEND_NULL_WREG;
                    end
                end

                ST_SEND_NULL_WREG: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_NULL_WREG;
                    end
                end

                ST_WAIT_NULL_WREG: begin
                    if (cmd_done) begin
                        wreg_ack_word <= cmd_rx_frame[143:120];
                        init_state    <= ST_CHECK_ACK;
                    end
                end

                ST_CHECK_ACK: begin
                    if (wreg_ack_word == 24'h418000) begin
                        // CLOCK written OK → Phase 3
                        cmd_tx_frame   <= {WREG_CFG_CMD,
                                           {GAIN1_VALUE[15:0], 8'h00},
                                           {GAIN2_VALUE[15:0], 8'h00},
                                           {CFG_VALUE[15:0],   8'h00}, 24'd0};
                        debug_tx_word0 <= WREG_CFG_CMD;
                        init_state     <= ST_SEND_WREG_CFG;
                    end
                    else begin
                        if (wreg_retry_cnt < MAX_WREG_RETRIES[3:0]) begin
                            wreg_retry_cnt <= wreg_retry_cnt + 1'b1;
                            retry_count    <= retry_count + 1'b1;
                            cmd_tx_frame   <= {WREG_CLOCK_CMD, WREG_CLOCK_DATA, 96'd0};
                            debug_tx_word0 <= WREG_CLOCK_CMD;
                            init_state     <= ST_SEND_WREG;
                        end
                        else begin
                            init_state <= ST_ERROR;
                            init_error <= 1'b1;
                        end
                    end
                end

                // ================================================
                // Phase 3: WREG GAIN1/GAIN2/CFG (3 data words)
                // ================================================

                ST_SEND_WREG_CFG: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_WREG_CFG;
                    end
                end

                ST_WAIT_WREG_CFG: begin
                    if (cmd_done) begin
                        cmd_tx_frame   <= 144'd0;
                        debug_tx_word0 <= 24'd0;
                        init_state     <= ST_SEND_NULL_WREG_CFG;
                    end
                end

                ST_SEND_NULL_WREG_CFG: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_NULL_WREG_CFG;
                    end
                end

                ST_WAIT_NULL_WREG_CFG: begin
                    if (cmd_done) begin
                        wreg_ack_word <= cmd_rx_frame[143:120];
                        init_state    <= ST_CHECK_ACK_CFG;
                    end
                end

                ST_CHECK_ACK_CFG: begin
                    if (wreg_ack_word == 24'h418000) begin
                        // GAIN/CFG written OK → Phase 4 read-back
                        rb_idx         <= 2'd0;
                        rb_cmd         <= RREG_CLOCK_CMD;
                        rb_expect      <= WREG_CLK_VALUE[15:0];
                        cmd_tx_frame   <= {RREG_CLOCK_CMD, 120'd0};
                        debug_tx_word0 <= RREG_CLOCK_CMD;
                        init_state     <= ST_RREG_SEND;
                    end
                    else begin
                        if (wreg_retry_cnt < MAX_WREG_RETRIES[3:0]) begin
                            wreg_retry_cnt <= wreg_retry_cnt + 1'b1;
                            retry_count    <= retry_count + 1'b1;
                            cmd_tx_frame   <= {WREG_CFG_CMD,
                                               {GAIN1_VALUE[15:0], 8'h00},
                                               {GAIN2_VALUE[15:0], 8'h00},
                                               {CFG_VALUE[15:0],   8'h00}, 24'd0};
                            debug_tx_word0 <= WREG_CFG_CMD;
                            init_state     <= ST_SEND_WREG_CFG;
                        end
                        else begin
                            init_state <= ST_ERROR;
                            init_error <= 1'b1;
                        end
                    end
                end

                // ================================================
                // Phase 4: read-back verification loop
                // ================================================

                ST_RREG_SEND: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_RREG;
                    end
                end

                ST_WAIT_RREG: begin
                    if (cmd_done) begin
                        cmd_tx_frame   <= 144'd0;
                        debug_tx_word0 <= 24'd0;
                        init_state     <= ST_SEND_NULL_RREG;
                    end
                end

                ST_SEND_NULL_RREG: begin
                    if (!spi_busy) begin
                        cmd_start  <= 1'b1;
                        init_state <= ST_WAIT_NULL_RREG;
                    end
                end

                ST_WAIT_NULL_RREG: begin
                    if (cmd_done) begin
                        // Capture the register value read back (16-bit
                        // register value lives in word0[23:8])
                        case (rb_idx)
                            2'd0: fs_hz          <= FS_HZ[31:0];   // CLOCK verified
                            2'd1: gain1_readback <= cmd_rx_frame[143:128];
                            2'd2: gain2_readback <= cmd_rx_frame[143:128];
                            2'd3: cfg_readback   <= cmd_rx_frame[143:128];
                            default: ;
                        endcase
                        init_state <= ST_CHECK_RREG;
                    end
                end

                ST_CHECK_RREG: begin
                    if (cmd_rx_frame[143:128] == rb_expect) begin
                        // Verified → next register or settle
                        if (rb_idx == 2'd3) begin
                            init_state <= ST_SETTLE;
                            cnt        <= {CNTR_W{1'b0}};
                        end
                        else begin
                            rb_idx <= rb_idx + 1'b1;
                            case (rb_idx)
                                2'd0: begin
                                    rb_cmd    <= RREG_GAIN1_CMD;
                                    rb_expect <= GAIN1_VALUE[15:0];
                                    cmd_tx_frame <= {RREG_GAIN1_CMD, 120'd0};
                                end
                                2'd1: begin
                                    rb_cmd    <= RREG_GAIN2_CMD;
                                    rb_expect <= GAIN2_VALUE[15:0];
                                    cmd_tx_frame <= {RREG_GAIN2_CMD, 120'd0};
                                end
                                2'd2: begin
                                    rb_cmd    <= RREG_CFG_CMD;
                                    rb_expect <= CFG_VALUE[15:0];
                                    cmd_tx_frame <= {RREG_CFG_CMD, 120'd0};
                                end
                                default: ;
                            endcase
                            debug_tx_word0 <= rb_cmd;
                            init_state     <= ST_RREG_SEND;
                        end
                    end
                    else begin
                        // Read-back mismatch → retry whole write phase
                        if (wreg_retry_cnt < MAX_WREG_RETRIES[3:0]) begin
                            wreg_retry_cnt <= wreg_retry_cnt + 1'b1;
                            retry_count    <= retry_count + 1'b1;
                            cmd_tx_frame   <= {WREG_CLOCK_CMD, WREG_CLOCK_DATA, 96'd0};
                            debug_tx_word0 <= WREG_CLOCK_CMD;
                            init_state     <= ST_SEND_WREG;
                        end
                        else begin
                            init_state <= ST_ERROR;
                            init_error <= 1'b1;
                        end
                    end
                end

                // ================================================
                // SETTLE / DONE / ERROR / RECONFIG
                // ================================================

                ST_SETTLE: begin
                    if (cnt >= SETTLE_CYCLES[CNTR_W-1:0] - 1'b1) begin
                        init_state    <= ST_DONE;
                        init_done     <= 1'b1;
                        stream_enable <= 1'b1;
                        cfg_id        <= cfg_id + 32'd1;
                    end
                    else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                ST_DONE: begin
                    init_done     <= 1'b1;
                    stream_enable <= 1'b1;
                    if (cfg_reconfig) begin
                        // Re-run phases 2..4: stop publishing, flush, re-verify.
                        init_done     <= 1'b0;
                        stream_enable <= 1'b0;
                        wreg_retry_cnt <= 4'd0;
                        cmd_tx_frame   <= {WREG_CLOCK_CMD, WREG_CLOCK_DATA, 96'd0};
                        debug_tx_word0 <= WREG_CLOCK_CMD;
                        init_state     <= ST_SEND_WREG;
                    end
                end

                ST_ERROR: begin
                    init_error <= 1'b1;
                end

                default: begin
                    init_state <= ST_ERROR;
                    init_error <= 1'b1;
                end

            endcase
        end
    end

endmodule

`default_nettype wire
