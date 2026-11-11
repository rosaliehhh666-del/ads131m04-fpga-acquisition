`timescale 1ns / 1ps
// 32 simultaneous signed 24-bit samples per channel, little endian.
module ads_screen_tx #(parameter integer FS_HZ = 2000)(
    input wire clk, rst_n, sample_valid, enabled,
    input wire [95:0] sample_data,
    output wire tx
);
    localparam HEADER = 24, PAYLOAD = 384, BODY = HEADER + PAYLOAD;
    localparam [63:0] MAGIC = 64'h2131345744534441; // "ADSDW41!"
    reg [95:0] samples [0:63];
    reg [4:0] capture_index, read_index;
    reg [3:0] lane_byte;
    reg [8:0] pos;
    reg sending, write_bank, frame_ready;
    reg [31:0] capture_first_index;
    reg [31:0] sequence_id, crc, first_index, sample_index;
    reg [7:0] data;
    wire ready;
    wire [31:0] final_crc = ~crc;
    function [31:0] crc_next;
        input [31:0] previous;
        input [7:0] value;
        integer i;
        reg [31:0] c;
        begin
            c = previous ^ {24'd0,value};
            for (i=0;i<8;i=i+1)
                c = (c >> 1) ^ (c[0] ? 32'hedb88320 : 32'd0);
            crc_next = c;
        end
    endfunction
    always @* begin
        if (pos < 8) data = MAGIC >> (8*pos[2:0]);
        else if (pos < 12) data = sequence_id >> (8*pos[1:0]);
        else if (pos < 16) data = FS_HZ >> (8*pos[1:0]);
        else if (pos < 20) data = first_index >> (8*pos[1:0]);
        else if (pos < HEADER) data = (pos == 20) ? 8'd32 : 8'd0;
        else if (pos < BODY) data = samples[{!write_bank, read_index}] >> (8*lane_byte);
        else data = final_crc >> (8*pos[1:0]);
    end
    uart_tx #(.CLK_FREQ(50000000),.BAUD(2000000)) u_uart (
        .clk(clk), .rst_n(rst_n), .tx_data(data), .tx_valid(sending),
        .tx_ready(ready), .tx(tx)
    );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            capture_index <= 0;
            read_index <= 0;
            lane_byte <= 0;
            pos <= 0;
            sending <= 0;
            write_bank <= 0;
            frame_ready <= 0;
            capture_first_index <= 0;
            sequence_id <= 0;
            sample_index <= 0;
            first_index <= 0;
            crc <= 32'hffffffff;
        end else begin
            if (!enabled) begin
                frame_ready <= 0;
                capture_index <= 0;
            end
            if (sample_valid) sample_index <= sample_index + 1'b1;
            if (enabled && !frame_ready && sample_valid) begin
                samples[{write_bank, capture_index}] <= sample_data;
                capture_index <= capture_index + 1'b1;
                if (capture_index == 0) capture_first_index <= sample_index;
                if (capture_index == 31) frame_ready <= 1;
            end
            if (enabled && !sending && frame_ready) begin
                // Switch banks only after the old frame has been handed to UART.
                write_bank <= !write_bank;
                first_index <= capture_first_index;
                frame_ready <= 0;
                sending <= 1;
            end
            if (sending && ready) begin
                if (pos < BODY) crc <= crc_next(crc,data);
                if (pos >= HEADER && pos < BODY) begin
                    if (lane_byte == 11) begin
                        lane_byte <= 0;
                        read_index <= read_index + 1'b1;
                    end else lane_byte <= lane_byte + 1'b1;
                end
                if (pos == BODY+3) begin
                    sending <= 0;
                    pos <= 0;
                    crc <= 32'hffffffff;
                    sequence_id <= sequence_id + 1'b1;
                end else pos <= pos + 1'b1;
            end
        end
    end
endmodule


// AA5, 50 MHz / 2 Mbaud, 8N1. Commands: A5 70 enable ~enable.
module ads_screen_control(input wire clk, rst_n, rx, output reg enabled);
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_rx;
    reg [4:0] ticks;
    reg [3:0] bit_pos;
    reg busy;
    reg [7:0] data;
    reg [1:0] cmd_pos;
    reg requested;
    reg [15:0] timeout_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_rx <= 2'b11;
            ticks <= 0;
            bit_pos <= 0;
            busy <= 0;
            data <= 0;
            cmd_pos <= 0;
            requested <= 0;
            enabled <= 1;
            timeout_count <= 0;
        end else begin
            sync_rx <= {sync_rx[0],rx};
            if (cmd_pos != 0) begin
                if (timeout_count == 49999) cmd_pos <= 0;
                else timeout_count <= timeout_count + 1'b1;
            end
            if (!busy) begin
                if (!sync_rx[1]) begin
                    busy <= 1;
                    bit_pos <= 0;
                    ticks <= 11;
                end
            end else if (ticks != 0) ticks <= ticks - 1'b1;
            else begin
                ticks <= 24;
                bit_pos <= bit_pos + 1'b1;
                if (bit_pos == 0 && sync_rx[1]) busy <= 0;
                else if (bit_pos >= 1 && bit_pos <= 8)
                    data <= {sync_rx[1], data[7:1]};
                else if (bit_pos == 9) begin
                    busy <= 0;
                    timeout_count <= 0;
                    if (!sync_rx[1]) cmd_pos <= 0;
                    else if (data == 8'ha5) cmd_pos <= 1;
                    else case (cmd_pos)
                        1: cmd_pos <= data == 8'h70 ? 2 : 0;
                        2: begin
                            cmd_pos <= data <= 1 ? 3 : 0;
                            requested <= data[0];
                        end
                        3: begin
                            if (data == (requested ? 8'hfe : 8'hff))
                                enabled <= requested;
                            cmd_pos <= 0;
                        end
                        default: cmd_pos <= 0;
                    endcase
                end
            end
        end
    end
endmodule

