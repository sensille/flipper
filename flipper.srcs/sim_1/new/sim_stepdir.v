`timescale 1ns / 1ps
`default_nettype none

module sim_stepdir();

reg clk_50mhz = 0;

wire leds_out;
wire leds_cs;
wire leds_clk;

wire rx;
reg tx = 1;

wire cs123;
reg sdo;
wire sdi;
wire sck;
wire [5:0] step;
wire [5:0] dir;

localparam BITRATE = 400000;
//localparam BITRATE = 9600;
localparam BITLENGTH = 1000000000 / BITRATE;

reg [7:0] endstop = 0;

flipper #(
	.BAUD(BITRATE),
	.NENDSTOP(8)
) u_flipper(
	.clk_50mhz(clk_50mhz),

	.led(),

	.rx(tx),
	.tx(rx),
	.cts(),

	// stepper interface
	.en(),
	.sck(sck),
	.cs123(cs123),
	.cs456(),
	.sdi(sdi),
	.sdo(sdo),
	.dir(dir),
	.step(step),

	.endstop(endstop),

	.fan_hotend(),
	.fan_part(),
	.hotend(),
	.bed(),
	.probe_servo(),
	.probe(1'b0),
	.fan_extruder(),

	// debug
	.debug1(),
	.debug2(),
	.debug3(),
	.debug4(),

	//
	.leds_out(leds_out),
	.leds_clk(leds_clk),
	.leds_cs(leds_cs)
);

task send;
input [7:0] data;
begin : sendtask
	integer i;

	tx = 0;	/* start bit */
	for (i = 0; i < 8; i = i + 1) begin
		#BITLENGTH tx = data[0];
		data = { 1'b0, data[7:1] };
	end
	#BITLENGTH tx = 1;	/* stop bit and idle */
	#BITLENGTH;
	#BITLENGTH;
end
endtask

initial begin: B_clk
	integer i;
	for (i = 0; i < 10000000; i = i + 1) begin
		clk_50mhz = 1;
		#10;
		clk_50mhz = 0;
		#10;
	end
end

initial begin: B_serial_data
	tx = 1;
	#1000;
	send(8'h0b);
	send(8'h10);
	send(8'h0f);	// config_stepper
	send(8'h01);	// oid
	send(8'h50);	// STEP0
	send(8'h40);	// DIR0
	send(8'h10);	// min_stop_interval=16
	send(8'h00);	// invert_step=0=16
	send(8'h1e);
	send(8'h15);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h0a);
	send(8'h11);
	send(8'h19);	// reset_step_clock
	send(8'h01);	// oid
	                // clock 0x4800 0100 1000 0000 0000
	send(8'h81);	// 01 0010000 0000000
	send(8'h90);
	send(8'h00);
	send(8'h6a);
	send(8'hb1);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h0a);
	send(8'h12);
	send(8'h15);	// queue_step
	send(8'h01);	// oid
	send(8'h50);	// interval
	send(8'h30);	// count
	send(8'h10);	// add=16
	send(8'h7b);
	send(8'h51);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h08);
	send(8'h13);
	send(8'h18);	// set_next_step_dir
	send(8'h01);	// oid
	send(8'h01);	// dir
	send(8'hc7);
	send(8'hc2);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h0a);
	send(8'h14);
	send(8'h15);	// queue_step
	send(8'h01);	// oid
	send(8'h50);	// interval
	send(8'h20);	// count
	send(8'h70);	// add=16
	send(8'h95);
	send(8'h3c);
	send(8'h7e);
end

endmodule
