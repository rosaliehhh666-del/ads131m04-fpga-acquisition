`timescale 1ns / 1ps
`default_nettype none

/*
 * crc8_check_120.v
 *
 * Parallel CRC-8 checker over 120 bits, used at the core-top level to verify
 * ADS131M04 stream frames WITHOUT touching the SPI driver.
 *
 * ADS131M04 CRC-8: polynomial x^8 + x^2 + x + 1 (0x07), init 0x00,
 * MSB-first, computed over the first five 24-bit words of a stream frame:
 *   {status[23:0], ch0[23:0], ch1[23:0], ch2[23:0], ch3[23:0]}
 * The transmitted CRC occupies the low 8 bits of the 6th word (crc_word[7:0]).
 *
 * check_en: 1 = compare and assert crc_err on mismatch
 *           0 = compute only (diagnostic), crc_err stays low
 *
 * The combinational CRC tree is a plain XOR reduction written as a for-loop
 * function; synthesis maps it to LUT logic (~6-8 LUT levels at 50 MHz, no
 * timing concern). The result is registered before use.
 */

module crc8_check_120 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        check_en,
    input  wire [23:0] status_word,
    input  wire [23:0] ch0,
    input  wire [23:0] ch1,
    input  wire [23:0] ch2,
    input  wire [23:0] ch3,
    input  wire [7:0]  crc_word_lsb,   // received CRC (low 8 bits of 6th word)
    input  wire        frame_valid,    // strobe when the frame is complete
    output reg  [7:0]  computed_crc,   // registered CRC over first 120 bits
    output reg         crc_err         // registered mismatch result
);

    // Serial CRC-8 update, MSB-first, poly 0x07 (x^8 + x^2 + x + 1).
    function [7:0] crc_next8;
        input [7:0] crc;
        input       bit_in;
        reg         feedback;
        begin
            feedback = crc[7] ^ bit_in;
            crc_next8[0] = feedback;
            crc_next8[1] = crc[0] ^ feedback;
            crc_next8[2] = crc[1] ^ feedback;
            crc_next8[3] = crc[2];
            crc_next8[4] = crc[3];
            crc_next8[5] = crc[4];
            crc_next8[6] = crc[5];
            crc_next8[7] = crc[6];
        end
    endfunction

    // Parallel CRC over the 120-bit payload (MSB first: bit 119 down to bit 0).
    function [7:0] crc_parallel;
        input [119:0] data;
        reg [7:0] crc;
        integer i;
        begin
            crc = 8'h00;
            for (i = 119; i >= 0; i = i - 1)
                crc = crc_next8(crc, data[i]);
            crc_parallel = crc;
        end
    endfunction

    wire [119:0] payload = {status_word, ch0, ch1, ch2, ch3};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            computed_crc <= 8'd0;
            crc_err      <= 1'b0;
        end else begin
            if (frame_valid) begin
                computed_crc <= crc_parallel(payload);
                if (check_en)
                    crc_err <= (crc_parallel(payload) != crc_word_lsb);
                else
                    crc_err <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
