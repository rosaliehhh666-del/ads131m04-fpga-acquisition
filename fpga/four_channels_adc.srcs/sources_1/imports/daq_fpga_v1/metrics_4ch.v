`timescale 1ns / 1ps
module metrics_4ch #(
    parameter integer LOG2_N = 11
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,

    input  wire signed [23:0] ch0,
    input  wire signed [23:0] ch1,
    input  wire signed [23:0] ch2,
    input  wire signed [23:0] ch3,

    output wire signed [23:0] mean0,
    output wire signed [23:0] mean1,
    output wire signed [23:0] mean2,
    output wire signed [23:0] mean3,

    output wire [24:0] vpp0,
    output wire [24:0] vpp1,
    output wire [24:0] vpp2,
    output wire [24:0] vpp3,

    output wire [23:0] rms0,
    output wire [23:0] rms1,
    output wire [23:0] rms2,
    output wire [23:0] rms3,

    output wire        metrics_valid
);

wire valid0, valid1, valid2, valid3;

channel_metrics #(.LOG2_N(LOG2_N)) u_m0 (
    .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .sample(ch0),
    .mean(mean0), .vpp(vpp0), .rms_ac(rms0), .metrics_valid(valid0)
);

channel_metrics #(.LOG2_N(LOG2_N)) u_m1 (
    .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .sample(ch1),
    .mean(mean1), .vpp(vpp1), .rms_ac(rms1), .metrics_valid(valid1)
);

channel_metrics #(.LOG2_N(LOG2_N)) u_m2 (
    .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .sample(ch2),
    .mean(mean2), .vpp(vpp2), .rms_ac(rms2), .metrics_valid(valid2)
);

channel_metrics #(.LOG2_N(LOG2_N)) u_m3 (
    .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .sample(ch3),
    .mean(mean3), .vpp(vpp3), .rms_ac(rms3), .metrics_valid(valid3)
);

/*
 * All four modules see the same sample_valid and have identical latency.
 */
assign metrics_valid = valid0 & valid1 & valid2 & valid3;

endmodule
