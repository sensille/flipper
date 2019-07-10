`timescale 1ns / 1ps
`default_nettype none

module framing #(
	parameter BAUD = 9600,
	parameter RING_BITS = 8,
	parameter HZ = 20000000
) (
	input wire clk,

	input wire rx,
	output wire tx,
	output wire cts,

	/*
	 * receive side
	 */
	output wire [7:0] msg_data,
	output wire msg_ready,
	input wire msg_rd_en,

	/* reset */
	input wire clr
);

localparam RING_SIZE = 1 << RING_BITS;

/*
 * sync rx into our clk domain
 */
reg rx_s1 = 0;
reg rx_sync = 0;
always @(posedge clk) begin
	rx_s1 <= rx;
	rx_sync <= rx_s1;
end

/*
 * UART instantiation
 */
wire [7:0] rx_data;
wire rx_ready;
reg [7:0] tx_data = 0;
reg tx_en = 0;
wire tx_transmitting;

localparam CLOCK_DIVIDE = HZ / BAUD / 4; /* 4 phases per bit */
uart #(
        .CLOCK_DIVIDE(CLOCK_DIVIDE)
) uart_u (
	.clk(clk),
	.rst(1'b0),
	.rx(rx_sync),
	.tx(tx),
	.transmit(tx_en),
	.tx_byte(tx_data),
	.received(rx_ready),
	.rx_byte(rx_data),
	.is_receiving(),
	.is_transmitting(tx_transmitting),
	.recv_error()
);

reg [7:0] recv_ring [RING_SIZE-1:0];
reg [RING_BITS-1:0] recv_temp_wptr = 0;
reg [RING_BITS-1:0] recv_wptr = 0;
reg [RING_BITS-1:0] recv_rptr = 0;
assign msg_data = recv_ring[recv_rptr];
assign msg_ready = recv_rptr != recv_wptr;

localparam RECV_EOF_CHAR = 8'h7e;

reg [7:0] crc16_in = 0;
reg [3:0] crc16_cnt = 0;
reg [15:0] crc16 = 0;
reg [7:0] recv_crc1 = 0;
reg [7:0] recv_crc2 = 0;
reg [7:0] recv_len = 0;
reg [7:0] recv_seq = 0;
reg [3:0] recv_next_seq = 0;
reg [2:0] recv_state;
localparam RST_SOF = 3'd0;	/* start of frame (== len byte) */
localparam RST_SEQ = 3'd1;	/* read sequence byte */
localparam RST_DATA = 3'd2;	/* read data */
localparam RST_CRC1 = 3'd3;	/* read crc1 */
localparam RST_CRC2 = 3'd4;	/* read crc2 */
localparam RST_EOF = 3'd5;	/* read sync byte (end of frame) */
localparam RST_NEED_SYNC = 3'd6;/* lost sync, error state */

assign cts = (recv_rptr == recv_temp_wptr + 1'b1);

/*
 * packet receive state machine
 */
always @(posedge clk) begin
	if (rx_ready) begin
		if (recv_state == RST_NEED_SYNC) begin
			if (rx_data == RECV_EOF_CHAR) begin
				recv_state <= RST_SOF;
			end
		end else if (recv_state == RST_SOF) begin 
			if (rx_data == RECV_EOF_CHAR) begin
				/*
				 * additional sync bytes at start of frame
				 * are ignored
				 */
			end else if (rx_data < 5 || rx_data >= 64) begin
				/* bad len, go hunting for sync byte */
				recv_state <= RST_NEED_SYNC;
			end else begin
				recv_state <= RST_SEQ;
				recv_len <= rx_data;
				crc16_in <= rx_data;
				crc16_cnt <= 8;
				crc16 <= 0;
				recv_temp_wptr <= recv_wptr;
			end
		end else if (recv_state == RST_SEQ) begin
			if (recv_len == 5)
				recv_state <= RST_CRC1; /* no data phase */
			else
				recv_state <= RST_DATA;
			recv_seq <= rx_data;
			crc16_in <= rx_data;
			crc16_cnt <= 8;
		end else if (recv_state == RST_DATA) begin
			recv_wptr <= recv_wptr + 1'b1;
			recv_ring[recv_wptr] <= rx_data;
			recv_len <= recv_len - 1;
			crc16_in <= rx_data;
			crc16_cnt <= 8;
			if (recv_len == 5) begin
				recv_state <= RST_CRC1;
			end
		end else if (recv_state == RST_CRC1) begin
			recv_crc1 <= rx_data;
			recv_state <= RST_CRC2;
		end else if (recv_state == RST_CRC2) begin
			recv_crc2 <= rx_data;
			recv_state <= RST_EOF;
		end else if (recv_state == RST_EOF &&
		             rx_data == RECV_EOF_CHAR) begin
			recv_state <= RST_SOF;
			if (crc16 != { recv_crc1, recv_crc2 }) begin
				/* bad crc, ignore frame */
			end else if (recv_seq[3:0] != recv_next_seq) begin
				/* bad seq, ignore frame */
			end else if (recv_seq[7:4] != 4'b0001) begin
				/* invalid upper bits of seq, ignore frame */
			end else begin
				/*
				 * commit the write pointer to make the frame
				 * available to the reader
				 */
				recv_wptr <= recv_temp_wptr;
				recv_next_seq <= recv_seq + 1'b1;
			end
			
		end else if (recv_state == RST_EOF) begin
			/* missing EOF, discard */
			recv_state <= RST_NEED_SYNC;
		end
	end
	if (crc16_cnt != 0) begin
		crc16 <= { 1'b0, crc16[15:1] };
		crc16[15] <= crc16_in[0] ^ crc16[0];
		crc16[13] <= crc16[14] ^ crc16_in[0] ^ crc16[0];
		crc16[0] <= crc16[1] ^ crc16_in[0] ^ crc16[0];
		crc16_cnt <= crc16_cnt - 1'b1;
		crc16_in <= { 1'b0, crc16_in[7:1] };
	end

	/* read interface */
	if (msg_rd_en && msg_ready) begin
		recv_rptr <= recv_rptr + 1'b1;
	end

	/* reset */
	if (clr)
		recv_wptr <= 0;
end

endmodule
