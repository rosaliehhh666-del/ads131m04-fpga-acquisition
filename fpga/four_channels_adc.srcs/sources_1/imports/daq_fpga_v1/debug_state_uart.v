`timescale 1ns / 1ps
`default_nettype none

/*
 * debug_state_uart.v  —  UART MUX with full ADC state diagnostics
 *
 * When ADC init is NOT done:
 *   Sends ASCII diagnostic line every 250 ms:
 *   "ST=02 R=0 ID=FFFFFF TX=000000 RX0=000000 RX1=000000 B=0 D=0\r\n"
 *
 * When ADC init IS done:
 *   Passes through VOFA+ JustFloat data from vofa_uart_tx.
 */

module debug_state_uart #(
    parameter integer CLK_FREQ      = 50_000_000,
    parameter integer BAUD          = 115_200,
    parameter integer HEARTBEAT_MS  = 250
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [3:0]  init_state,
    input  wire [3:0]  retry_cnt,
    input  wire [23:0] id_word,
    input  wire [23:0] spi_tx_word0,
    input  wire [23:0] spi_rx_word0,
    input  wire [23:0] spi_rx_word1,
    input  wire        spi_busy,
    input  wire        spi_cmd_done,
    input  wire        init_done,
    input  wire        vofa_uart_tx,

    output wire        uart_tx
);

    assign uart_tx = init_done ? vofa_uart_tx : dbg_uart_tx;

    wire dbg_uart_tx;

    // ============================================================
    // 250 ms heartbeat timer
    // ============================================================
    localparam integer HB_CYCLES = (CLK_FREQ / 1000) * HEARTBEAT_MS;
    localparam integer HB_W =
        (HB_CYCLES <= 2) ? 1 : $clog2(HB_CYCLES);

    reg [HB_W-1:0] hb_counter;
    reg            hb_trigger;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_counter <= {HB_W{1'b0}};
            hb_trigger <= 1'b0;
        end
        else if (hb_counter == HB_CYCLES[HB_W-1:0] - 1'b1) begin
            hb_counter <= {HB_W{1'b0}};
            hb_trigger <= 1'b1;
        end
        else begin
            hb_counter <= hb_counter + 1'b1;
            hb_trigger <= 1'b0;
        end
    end

    // ============================================================
    // Hex nibble to ASCII
    // ============================================================
    function [7:0] hex2ascii;
        input [3:0] nibble;
        begin
            hex2ascii = (nibble < 4'd10) ?
                (8'h30 + nibble) :
                (8'h37 + nibble);
        end
    endfunction

    // ============================================================
    // Message format (61 bytes total):
    //   ST=XX R=X ID=XXXXXX TX=XXXXXX RX0=XXXXXX RX1=XXXXXX B=X D=X\r\n
    //
    //  0-4    ST=XX       (5)
    //  5      space       (1)
    //  6-8    R=X         (3)
    //  9      space       (1)
    // 10-18   ID=XXXXXX   (9)
    // 19      space       (1)
    // 20-28   TX=XXXXXX   (9)
    // 29      space       (1)
    // 30-38   RX0=XXXXXX  (9)
    // 39      space       (1)
    // 40-48   RX1=XXXXXX  (9)
    // 49      space       (1)
    // 50-52   B=X         (3)
    // 53      space       (1)
    // 54-56   D=X         (3)
    // 57-58   \r\n        (2)
    // ============================================================
    localparam integer MSG_LEN = 59;

    reg [7:0] msg_buf [0:MSG_LEN-1];
    reg [5:0] byte_idx;
    reg       tx_active;
    reg [7:0] tx_byte;
    reg       tx_valid;
    wire      tx_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_idx  <= 6'd0;
            tx_active <= 1'b0;
            tx_byte   <= 8'h00;
            tx_valid  <= 1'b0;
        end
        else begin
            if (!tx_active && hb_trigger) begin
                // ST=XX
                msg_buf[0]  <= 8'h53;  // 'S'
                msg_buf[1]  <= 8'h54;  // 'T'
                msg_buf[2]  <= 8'h3D;  // '='
                msg_buf[3]  <= 8'h30;  // state hi nibble
                msg_buf[4]  <= hex2ascii(init_state);
                msg_buf[5]  <= 8'h20;  // ' '
                // R=X
                msg_buf[6]  <= 8'h52;  // 'R'
                msg_buf[7]  <= 8'h3D;  // '='
                msg_buf[8]  <= hex2ascii(retry_cnt);
                msg_buf[9]  <= 8'h20;  // ' '
                // ID=XXXXXX
                msg_buf[10] <= 8'h49;  // 'I'
                msg_buf[11] <= 8'h44;  // 'D'
                msg_buf[12] <= 8'h3D;  // '='
                msg_buf[13] <= hex2ascii(id_word[23:20]);
                msg_buf[14] <= hex2ascii(id_word[19:16]);
                msg_buf[15] <= hex2ascii(id_word[15:12]);
                msg_buf[16] <= hex2ascii(id_word[11:8]);
                msg_buf[17] <= hex2ascii(id_word[7:4]);
                msg_buf[18] <= hex2ascii(id_word[3:0]);
                msg_buf[19] <= 8'h20;  // ' '
                // TX=XXXXXX
                msg_buf[20] <= 8'h54;  // 'T'
                msg_buf[21] <= 8'h58;  // 'X'
                msg_buf[22] <= 8'h3D;  // '='
                msg_buf[23] <= hex2ascii(spi_tx_word0[23:20]);
                msg_buf[24] <= hex2ascii(spi_tx_word0[19:16]);
                msg_buf[25] <= hex2ascii(spi_tx_word0[15:12]);
                msg_buf[26] <= hex2ascii(spi_tx_word0[11:8]);
                msg_buf[27] <= hex2ascii(spi_tx_word0[7:4]);
                msg_buf[28] <= hex2ascii(spi_tx_word0[3:0]);
                msg_buf[29] <= 8'h20;  // ' '
                // RX0=XXXXXX
                msg_buf[30] <= 8'h52;  // 'R'
                msg_buf[31] <= 8'h58;  // 'X'
                msg_buf[32] <= 8'h30;  // '0'
                msg_buf[33] <= 8'h3D;  // '='
                msg_buf[34] <= hex2ascii(spi_rx_word0[23:20]);
                msg_buf[35] <= hex2ascii(spi_rx_word0[19:16]);
                msg_buf[36] <= hex2ascii(spi_rx_word0[15:12]);
                msg_buf[37] <= hex2ascii(spi_rx_word0[11:8]);
                msg_buf[38] <= hex2ascii(spi_rx_word0[7:4]);
                msg_buf[39] <= hex2ascii(spi_rx_word0[3:0]);
                msg_buf[40] <= 8'h20;  // ' '
                // RX1=XXXXXX
                msg_buf[41] <= 8'h52;  // 'R'
                msg_buf[42] <= 8'h58;  // 'X'
                msg_buf[43] <= 8'h31;  // '1'
                msg_buf[44] <= 8'h3D;  // '='
                msg_buf[45] <= hex2ascii(spi_rx_word1[23:20]);
                msg_buf[46] <= hex2ascii(spi_rx_word1[19:16]);
                msg_buf[47] <= hex2ascii(spi_rx_word1[15:12]);
                msg_buf[48] <= hex2ascii(spi_rx_word1[11:8]);
                msg_buf[49] <= hex2ascii(spi_rx_word1[7:4]);
                msg_buf[50] <= hex2ascii(spi_rx_word1[3:0]);
                msg_buf[51] <= 8'h20;  // ' '
                // B=X
                msg_buf[52] <= 8'h42;  // 'B'
                msg_buf[53] <= 8'h3D;  // '='
                msg_buf[54] <= hex2ascii({3'b0, spi_busy});
                msg_buf[55] <= 8'h20;  // ' '
                // D=X
                msg_buf[56] <= 8'h44;  // 'D'
                msg_buf[57] <= 8'h3D;  // '='
                msg_buf[58] <= hex2ascii({3'b0, spi_cmd_done});
                // Note: \r\n omitted for now; 59 bytes exactly fills 0..58

                byte_idx  <= 6'd0;
                tx_active <= 1'b1;
                tx_byte   <= 8'h53;  // 'S'
                tx_valid  <= 1'b1;
            end
            else if (tx_active && tx_valid && tx_ready) begin
                if (byte_idx == MSG_LEN - 1) begin
                    tx_active <= 1'b0;
                    tx_valid  <= 1'b0;
                    byte_idx  <= 6'd0;
                end
                else begin
                    byte_idx <= byte_idx + 1'b1;
                    tx_byte  <= msg_buf[byte_idx + 1'b1];
                    tx_valid <= 1'b1;
                end
            end
        end
    end

    // ============================================================
    // Proven UART transmitter
    // ============================================================
    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_uart_tx_debug (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (tx_byte),
        .tx_valid (tx_valid),
        .tx_ready (tx_ready),
        .tx       (dbg_uart_tx)
    );

endmodule

`default_nettype wire
