`timescale 1ns/1ps
`default_nettype none
/* Simulation-only MCU Mode-0 master. No register backdoor access.\n * This is a support BFM called by tb_ads_fpga_mcu; do NOT set it as simulation top.\n * Default SCLK is 2 MHz. Optional nonnegative edge jitter lengthens halves.
 * Calls are serial/non-reentrant; one bus master owns this instance.
 */
module mcu_spi_master_bfm(
    output reg cs_n,
    output reg sclk,
    output reg mosi,
    input wire miso
);
    integer half_ns, jitter_ns, bit_transfers, ignored_plusarg;
    initial begin
        cs_n=1; sclk=0; mosi=0; bit_transfers=0;
        half_ns=250; jitter_ns=0;
        ignored_plusarg=$value$plusargs("MCU_HALF_NS=%d",half_ns);
        ignored_plusarg=$value$plusargs("MCU_JITTER_NS=%d",jitter_ns);
        if (half_ns < 250 || jitter_ns < 0)
            $fatal(1,"MCU_TIMING: HALF_NS must be >=250, JITTER_NS >=0");
    end
    task automatic idle;
        begin sclk=0; mosi=0; cs_n=1; #401; end
    endtask
    task automatic begin_frame;
        begin
            if (cs_n !== 1'b1) $fatal(1,"MCU_BFM: begin while CS already active");
            sclk=0; mosi=0; #237; cs_n=0; #277;
        end
    endtask
    task automatic end_frame;
        begin
            if (sclk !== 1'b0) $fatal(1,"MCU_BFM: CS changes require idle SCLK");
            #277; cs_n=1; #1;
            if (miso !== 1'bz) $fatal(1,"MCU_MISO_RELEASE: output not Hi-Z on deselect");
            #400;
        end
    endtask
    task automatic bit_xfer(input tx, output rx);
        time d;
        begin
            d=half_ns + ((bit_transfers % 3) * jitter_ns);
            mosi=tx; #(d); sclk=1; #1;
            if (miso !== 1'b0 && miso !== 1'b1)
                $fatal(1,"MCU_MISO_XZ: undriven/unknown data while selected");
            rx=miso;
            #(d-1); sclk=0; bit_transfers=bit_transfers+1;
        end
    endtask
    task automatic xfer_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        reg bit_rx;
        begin
            for (b=7; b>=0; b=b-1) begin
                bit_xfer(tx[b],bit_rx); rx[b]=bit_rx;
            end
        end
    endtask
    task automatic read_data_word(output [31:0] word_value);
        reg [7:0] a,b,c,d;
        begin
            xfer_byte(0,a); xfer_byte(0,b); xfer_byte(0,c); xfer_byte(0,d);
            word_value={a,b,c,d};
        end
    endtask
    task automatic write_data_word(input [31:0] word_value);
        reg [7:0] ignored;
        begin
            xfer_byte(word_value[31:24],ignored); xfer_byte(word_value[23:16],ignored);
            xfer_byte(word_value[15:8],ignored); xfer_byte(word_value[7:0],ignored);
        end
    endtask
    task automatic read_reg(input [7:0] address, output [31:0] value);
        reg [7:0] ignored;
        begin
            begin_frame(); xfer_byte(8'ha0,ignored); xfer_byte(address,ignored);
            read_data_word(value); end_frame();
        end
    endtask
    task automatic legacy_read(input [7:0] address, output [31:0] value);
        reg [7:0] ignored;
        begin
            begin_frame(); xfer_byte(address,ignored); read_data_word(value); end_frame();
        end
    endtask
    task automatic write_reg(input [7:0] address, input [31:0] value);
        reg [7:0] ignored;
        begin
            begin_frame(); xfer_byte(8'ha1,ignored); xfer_byte(address,ignored);
            write_data_word(value); end_frame();
        end
    endtask
endmodule
`default_nettype wire
