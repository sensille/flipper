`timescale 1ns / 1ps
`default_nettype none

module sim_top();

reg clk_50mhz = 0;

wire leds_out;
wire leds_cs;
wire leds_clk;

wire rx;
reg tx = 1;
wire rx2;
reg tx2 = 1;

wire cs123;
reg sdo;
wire sdi;
wire sck;

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
	.dir(),
	.step(),

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
	send(8'h07);
	send(8'h10);
	send(8'haa);
	send(8'hbb);
	send(8'h72);
	send(8'h8b);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h08);
	send(8'h11);
	send(8'h21);
	send(8'h22);
	send(8'h23);
	send(8'hbb);
	send(8'h41);
	send(8'h7e);
	#(BITLENGTH * 10);
end

endmodule
