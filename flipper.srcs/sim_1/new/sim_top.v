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
	send(8'h88);
	send(8'h42);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h08);
	send(8'h11);
	send(8'h21);
	send(8'h22);
	send(8'h23);
	send(8'hef);
	send(8'h4f);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h0a);
	send(8'h12);
	// identify
	send(8'h01);
	// 3e6fd
	// 1 00 01111 1 1001101 0 1111101
	// 1 00 0111110011011111101
	// 1 00 011 1110 0110 1111 1101
	send(8'h8f);
	send(8'hcd);
	send(8'h7d);
	// 25
	send(8'h25);
	send(8'h87);
	send(8'h1b);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h09);
	send(8'h13);
	// identify
	send(8'h01);
	// 490 == 0x1ea
	// 00011 1101010
	send(8'h83);
	send(8'h6a);
	send(8'h30);
	send(8'hd2);
	send(8'h1a);
	send(8'h7e);
	#(BITLENGTH * 200);
	send(8'h06);
	send(8'h14);
	// get_config
	send(8'h02);
	send(8'h3e);
	send(8'h09);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h0b);
	send(8'h15);
	// set_config, crc = 0x3e
	// 0001101 0101100 0011001 0111010 1111010
	// 000 1101 0101  1000 0110  0101 1101  0111 1010
	//      d    5      8    6    5    d      7   a
	//      0xd5865d7a
	send(8'h04);
	send(8'h8d);
	send(8'hac);
	send(8'h99);
	send(8'hba);
	send(8'h7a);
	send(8'h43);
	send(8'haa);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h06);
	send(8'h16);
	// get_clock
	send(8'h05);
	send(8'h79);
	send(8'h06);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h08);
	send(8'h17);
	// set_digital_out
	send(8'h0b);
	send(8'h00);
	send(8'h01);
	send(8'hc6);
	send(8'h07);
	send(8'h7e);
	#(BITLENGTH * 10);
	send(8'h0f);
	send(8'h18);
	send(8'h0d); // config_soft_pwm_out
	send(8'h11); // oid
	send(8'h80); // pin PWM0
	send(8'h60);
	send(8'h33); // cycle ticks
	send(8'h01); // vaule
	send(8'h00); // default
	send(8'h81); // max duration
	send(8'h80);
	send(8'h00);
	send(8'he8);
	send(8'h33);
	send(8'h7e);
	#(BITLENGTH * 200);
	send(8'h0b);
	send(8'h19);
	send(8'h0e); // schedule_soft_pwm_out
	send(8'h11); // oid
	send(8'h84); // clock 0x13600 000100 1101100 0000000
	send(8'hec);
	send(8'h00);
	send(8'h22); // on_ticks
	send(8'h58);
	send(8'h26);
	send(8'h7e);
	#(BITLENGTH * 400);
	send(8'h0d);
	send(8'h1a);
	send(8'h0e); // schedule_soft_pwm_out
	send(8'h11); // oid
	             // clock 0xabcd0123 1010 1011 1100 1101 0000 0001 0010 0011
	send(8'h8a); // clock 0xabcd0123 1010 1011110 0110100 0000010 0100011
	send(8'hde);
	send(8'hb4);
	send(8'h82);
	send(8'h23);
	send(8'h22); // on_ticks
	send(8'h7b);
	send(8'hec);
	send(8'h7e);
	#(BITLENGTH * 10);
end

endmodule
