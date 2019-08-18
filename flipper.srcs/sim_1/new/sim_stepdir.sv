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

task update_crc;
inout [15:0] crc16;
input [7:0] data;
begin: update_crc_task
	integer i;
	reg [15:0] old_crc16;
	for (i = 0; i < 8; i = i + 1) begin
		/* crc16 CCITT, reflect in, reflect out, init 0xffff */
		old_crc16 = crc16;
		crc16 = { 1'b0, crc16[15:1] };
		crc16[3] = data[0] ^ old_crc16[4] ^ old_crc16[0];
		crc16[10] = data[0] ^ old_crc16[11] ^ old_crc16[0];
		crc16[15] = data[0] ^ old_crc16[0];
		data = { 1'b0, data[7:1] };
	end
end
endtask

reg [3:0] seq = 0;
task send_packet;
input len;
integer len;
input [7:0] data [];
begin: send_packet_task
	integer i;
	static reg [15:0] crc16 = 16'hffff;

	send(len + 5);
	update_crc(crc16, len + 5);
	send({ 4'b0001, seq });
	update_crc(crc16, { 4'b0001, seq });
	seq = seq + 1;
	for (i = 0; i < len; i = i + 1) begin
		send(data[i]);
		update_crc(crc16, data[i]);
	end
	send(crc16[15:8]);
	send(crc16[7:0]);
	send(8'h7e);
end
endtask

task encode_arg;
input integer sv;
output integer len;
output [7:0] data [5];
begin: encode_arg_task
	automatic integer unsigned v = sv;
	automatic integer ix = 0;

	if (sv < (3<<5)  && sv >= -(1<<5))
		len = 1;
	else if (sv < (3<<12) && sv >= -(1<<12))
		len = 2;
	else if (sv < (3<<19) && sv >= -(1<<19))
		len = 3;
	else if (sv < (3<<26) && sv >= -(1<<26))
		len = 4;
	else
		len = 5;

	if (len >= 5) begin
		data[ix] = (v >> 28) | 8'h80;
		ix = ix + 1;
	end
	if (len >= 4) begin
		data[ix] = ((v >> 21) & 8'h7f) | 8'h80;
		ix = ix + 1;
	end
	if (len >= 3) begin
		data[ix] = ((v >> 14) & 8'h7f) | 8'h80;
		ix = ix + 1;
	end
	if (len >= 2) begin
		data[ix] = ((v >> 7) & 8'h7f) | 8'h80;
		ix = ix + 1;
	end
	data[ix] = v & 8'h7f;
end
endtask

task send_encoded;
input integer len;
input integer data [];
begin: send_encoded_task
	integer i;
	integer j;
	integer outlen;
	automatic integer ix = 0;
	reg [7:0] outdata [5];
	reg [7:0] packet [64];

	for (i = 0; i < len; i = i + 1) begin
		encode_arg(data[i], outlen, outdata);
$display("%d %h", outlen, " ", outdata);
		for (j = 0; j < outlen; j = j + 1) begin
			packet[ix] = outdata[j];
			ix = ix + 1;
		end
	end
	send_packet(ix, packet);
	#(BITLENGTH * 10);
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
	send_encoded( 6, {
		15,	// config_stepper
		1,	// oid
		8'h50,	// STEP0
		8'h40,	// DIR0
		16,	// min_stop_interval=16
		0 	// invert_step=0=16
	} );
	send_encoded( 3, {
		25, 	// reset_step_clock
		1,	// oid
		'h4800	// clock
	} );
	send_encoded( 5, {
		21, 	// queue_step
		1,	// oid
		80,	// interval
		48,	// count
		16	// add
	} );
	send_encoded( 3, {
		24, 	// set_next_step_dir
		1,	// oid
		1	// dir
	} );
	send_encoded( 5, {
		21, 	// queue_step
		1,	// oid
		80,	// interval
		32,	// count
		-2	// add
	} );
	#(BITLENGTH * 200);
	send_encoded( 2, {
		22, 	// stepper_get_position
		1	// oid
	} );
end

endmodule
