`timescale 1ns/1ps
`default_nettype none
/* Simulation-only subset of XPM synchronous FWFT FIFO used by this project.\n * For Vivado/XSim project-mode simulation, prefer AMD's native XPM library and DO NOT add this file.\n * Functional queue/count/full/empty, reset-busy, over/underflow and FWFT data.
 * NOT the AMD XPM simulation library: no ECC, sleep, CDC, timing or latency
 * equivalence. Never compile together with xpm_fifo.sv/vendor_compile_stubs.v.
 */
module xpm_fifo_sync #(
    parameter FIFO_MEMORY_TYPE="block", ECC_MODE="no_ecc",
    parameter integer FIFO_WRITE_DEPTH=4096, WRITE_DATA_WIDTH=96,
    parameter integer WR_DATA_COUNT_WIDTH=13, PROG_FULL_THRESH=4080,
    parameter integer FULL_RESET_VALUE=0,
    parameter READ_MODE="fwft",
    parameter integer FIFO_READ_LATENCY=0,
    parameter integer READ_DATA_WIDTH=96, RD_DATA_COUNT_WIDTH=13,
    parameter integer PROG_EMPTY_THRESH=16,
    parameter DOUT_RESET_VALUE="0", USE_ADV_FEATURES="1707",
    parameter integer WAKEUP_TIME=0
)(
    input wire sleep, rst, wr_clk, wr_en,
    input wire [WRITE_DATA_WIDTH-1:0] din,
    output wire full,
    output reg overflow,
    output wire [WR_DATA_COUNT_WIDTH-1:0] wr_data_count,
    output wire wr_rst_busy, almost_full, prog_full,
    output reg wr_ack,
    input wire rd_en,
    output wire [READ_DATA_WIDTH-1:0] dout,
    output wire empty,
    output reg underflow,
    output wire [RD_DATA_COUNT_WIDTH-1:0] rd_data_count,
    output wire rd_rst_busy, almost_empty, prog_empty, data_valid,
    input wire injectsbiterr, injectdbiterr,
    output wire sbiterr, dbiterr
);
    reg [WRITE_DATA_WIDTH-1:0] memory [0:FIFO_WRITE_DEPTH-1];
    integer read_ptr, write_ptr, count, busy_left;
    wire blocked = rst || (busy_left != 0);
    wire do_write = wr_en && !full && !blocked;
    wire do_read = rd_en && !empty && !blocked;
    assign full = (count == FIFO_WRITE_DEPTH);
    assign empty = (count == 0);
    assign data_valid = !empty && !blocked;
    assign dout = data_valid ? memory[read_ptr] : {READ_DATA_WIDTH{1'b0}};
    assign wr_data_count = count[WR_DATA_COUNT_WIDTH-1:0];
    assign rd_data_count = count[RD_DATA_COUNT_WIDTH-1:0];
    assign wr_rst_busy = blocked;
    assign rd_rst_busy = blocked;
    assign almost_full = (count >= FIFO_WRITE_DEPTH-1);
    assign prog_full = (count >= PROG_FULL_THRESH);
    assign almost_empty = (count <= 1);
    assign prog_empty = (count <= PROG_EMPTY_THRESH);
    assign sbiterr=1'b0;
    assign dbiterr=1'b0;
    initial begin
        read_ptr=0; write_ptr=0; count=0; busy_left=3;
        overflow=0; underflow=0; wr_ack=0;
        if (READ_MODE != "fwft" || FIFO_READ_LATENCY != 0 ||
            READ_DATA_WIDTH != WRITE_DATA_WIDTH || ECC_MODE != "no_ecc")
            $fatal(1,"FIFO_MODEL_CONFIG: only equal-width, latency-0, no-ECC FWFT supported");
    end
    always @(posedge wr_clk) begin
        if (rst) begin
            read_ptr<=0; write_ptr<=0; count<=0; busy_left<=3;
            overflow<=0; underflow<=0; wr_ack<=0;
        end else if (busy_left != 0) begin
            busy_left<=busy_left-1;
            overflow<=0; underflow<=0; wr_ack<=0;
            if (wr_en || rd_en) $fatal(1,"FIFO_RESET_BUSY: wrapper allowed access");
        end else begin
            if (sleep || injectsbiterr || injectdbiterr)
                $fatal(1,"FIFO_MODEL_UNSUPPORTED: sleep/ECC injection requested");
            overflow<=wr_en && full;
            underflow<=rd_en && empty;
            wr_ack<=do_write;
            if (do_write) begin
                memory[write_ptr]<=din;
                write_ptr<=(write_ptr == FIFO_WRITE_DEPTH-1) ? 0 : write_ptr+1;
            end
            if (do_read)
                read_ptr<=(read_ptr == FIFO_WRITE_DEPTH-1) ? 0 : read_ptr+1;
            case ({do_write,do_read})
                2'b10: count<=count+1;
                2'b01: count<=count-1;
                default: begin end
            endcase
            if (count < 0 || count > FIFO_WRITE_DEPTH) $fatal(1,"FIFO_COUNT_RANGE");
        end
    end
endmodule
`default_nettype wire
