`timescale 1ns / 1ps
`default_nettype none

module flipper #(
	parameter NENDSTOP = 8,
	parameter BAUD = 9600
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
	output wire [5:0] dir,
	output wire [5:0] step,

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
assign debug3 = 0;
assign debug4 = 0;

wire clk;
clk u_clk(
	.clk_50mhz(clk_50mhz),
	.clk(clk)		// 20 MHz
);

assign fan_hotend = 0;
assign fan_part = 0;
assign hotend = 0;
assign bed = 0;
assign probe_servo = 0;
assign fan_extruder = 0;
assign en = 1;
assign sck = 0;
assign cs123 = 1;
assign cs456 = 1;
assign sdi = 0;
assign dir = 0;
assign step = 0;

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

command #(
	.LEN_BITS(LEN_BITS),
	.LEN_FIFO_BITS(LEN_FIFO_BITS),
        .CMD_ACKNAK(CMD_ACKNAK)
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
	.send_ring_full()
);

reg [31:0] led_cnt = 0;
always @(posedge clk)
	led_cnt = led_cnt + 1;
assign led = led_cnt[22];

wire [255:0] leds;

assign leds[1:0] = { rx, tx };
assign leds[31:2] = 0;
assign leds[54:32] = 0;
assign leds[63:57] = 0;
assign leds[223:72] = 0;
assign leds[255:224] = led_cnt;

led7219 led7219_u(
	.clk(clk),
	.data(leds),
	.leds_out(leds_out),
	.leds_clk(leds_clk),
	.leds_cs(leds_cs)
);

endmodule
