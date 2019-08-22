`timescale 1ns / 1ps
`default_nettype none

module flipper #(
	parameter NENDSTOP = 8,
	parameter BAUD = 250000,
	parameter NSTEPDIR = 6
) (
	input wire clk_50mhz,

	output wire led,

	// to host
	output wire tx,
	input wire rx,
	output wire cts,

	// stepper interface
	output wire en,
	output wire sck,
	output wire cs123,
	output wire cs456,
	output wire sdi,
	input wire sdo,
	output wire [NSTEPDIR-1:0] dir,
	output wire [NSTEPDIR-1:0] step,

	// endstops
	input wire [NENDSTOP-1:0] endstop,

	// control
	output wire fan_hotend,
	output wire fan_part,
	output wire hotend,
	output wire bed,
	output wire probe_servo,
	input wire probe,
	output wire fan_extruder,

	// LED matrix
	output wire leds_out,
	output wire leds_clk,
	output wire leds_cs,

	// direct debug
	output wire debug1,
	output wire debug2,
	output wire debug3,
	output wire debug4
);

assign debug1 = rx;
assign debug2 = tx;
assign debug3 = fan_hotend;
assign debug4 = fan_part;

reg [63:0] clock = 0;
wire clk;
clk u_clk(
	.clk_50mhz(clk_50mhz),
	.clk(clk)		// 20 MHz
);

assign hotend = 0;
assign bed = 0;
assign sck = 0;
assign cs123 = 1;
assign cs456 = 1;
assign sdi = 0;

wire [7:0] msg_data;
wire msg_ready;
wire msg_rd_en;
/* max length of a packet MCU -> host */
localparam LEN_BITS = 6;
/* address width of fifo */
localparam LEN_FIFO_BITS = 5;
/* size of receive ring (2^x) */
localparam RING_BITS = 8;
localparam CMD_ACKNAK = 8'h7b;
wire [LEN_BITS-1:0] send_fifo_data;
wire send_fifo_wr_en;
wire [7:0] send_ring_data;
wire send_ring_wr_en;
framing #(
	.BAUD(BAUD),
	.RING_BITS(RING_BITS),
	.LEN_BITS(LEN_BITS),
	.LEN_FIFO_BITS(LEN_FIFO_BITS),
	.HZ(20000000),
        .CMD_ACKNAK(CMD_ACKNAK)
) u_framing (
	.clk(clk),

	.rx(rx),
	.tx(tx),
	.cts(cts),

	/*
	 * receive side
	 */
	.msg_data(msg_data),
	.msg_ready(msg_ready),
	.msg_rd_en(msg_rd_en),

	/*
	 * send side
	 */
	.send_fifo_wr_en(send_fifo_wr_en),
	.send_fifo_data(send_fifo_data),
	.send_fifo_full(),

	/* ring buffer input */
	.send_ring_data(send_ring_data),
	.send_ring_wr_en(send_ring_wr_en),
	.send_ring_full(),

	/* reset */
	.clr(1'b0)
);

localparam NGPIO = 4;
localparam NPWM = 6;
wire [NGPIO-1:0] gpio;
wire [NPWM-1:0] pwm;
wire [63:0] cmd_debug;

command #(
	.LEN_BITS(LEN_BITS),
	.LEN_FIFO_BITS(LEN_FIFO_BITS),
        .CMD_ACKNAK(CMD_ACKNAK),
	.NGPIO(NGPIO),
	.NPWM(NPWM),
	.NSTEPDIR(NSTEPDIR),
	.NENDSTOP(NENDSTOP)
) u_command (
	.clk(clk),

	/*
	 * receive side
	 */
	.msg_data(msg_data),
	.msg_ready(msg_ready),
	.msg_rd_en(msg_rd_en),

	/*
	 * send side
	 */
	.send_fifo_wr_en(send_fifo_wr_en),
	.send_fifo_data(send_fifo_data),
	.send_fifo_full(),

	/* ring buffer input */
	.send_ring_data(send_ring_data),
	.send_ring_wr_en(send_ring_wr_en),
	.send_ring_full(),

	/* global clock */
	.clock(clock),

	/* I/O */
	.gpio(gpio),
	.pwm(pwm),
	.step(step),
	.dir(dir),
	.endstop(endstop),

	.debug(cmd_debug)
);

assign en = gpio[0];
assign fan_hotend = pwm[0];
assign fan_part = pwm[1];
//assign hotend = pwm[2];
//assign bed = pwm[3];
assign fan_extruder = pwm[4];
assign probe_servo = pwm[5];

always @(posedge clk)
	clock = clock + 1;
assign led = clock[22];

wire [255:0] leds;

assign leds[1:0] = { rx, tx };
assign leds[5:2] = gpio;
assign leds[31:6] = 0;
assign leds[32+NPWM-1:32] = pwm;
assign leds[54:32+NPWM] = 0;
assign leds[63:57] = 0;
assign leds[127:64] = cmd_debug;
assign leds[191:128] = 0;
assign leds[255:192] = clock;

led7219 led7219_u(
	.clk(clk),
	.data(leds),
	.leds_out(leds_out),
	.leds_clk(leds_clk),
	.leds_cs(leds_cs)
);

endmodule
