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
 * Initialize to one (idle)
 */
reg rx_s1 = 1;
reg rx_sync = 1;
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
localparam RST_SOF = 3'd0;	/* start of frame (== len byte) */
localparam RST_SEQ = 3'd1;	/* read sequence byte */
localparam RST_DATA = 3'd2;	/* read data */
localparam RST_CRC1 = 3'd3;	/* read crc1 */
localparam RST_CRC2 = 3'd4;	/* read crc2 */
localparam RST_EOF = 3'd5;	/* read sync byte (end of frame) */
localparam RST_NEED_SYNC = 3'd6;/* lost sync, error state */
reg [2:0] recv_state = RST_SOF;

assign cts = (recv_rptr == recv_temp_wptr + 1'b1);

reg send_acknak = 0;
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
				send_acknak <= 1;
			end else begin
				recv_state <= RST_SEQ;
				recv_len <= rx_data;
				crc16_in <= rx_data;
				crc16_cnt <= 8;
				crc16 <= 16'hffff;
				//crc16 <= 16'h0000;
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
			recv_temp_wptr <= recv_temp_wptr + 1'b1;
			recv_ring[recv_temp_wptr] <= rx_data;
			recv_len <= recv_len - 1;
			crc16_in <= rx_data;
			crc16_cnt <= 8;
			if (recv_len == 6) begin
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
			send_acknak <= 1;
		end else if (recv_state == RST_EOF) begin
			/* missing EOF, discard */
			recv_state <= RST_NEED_SYNC;
			send_acknak <= 1;
		end
	end
	if (crc16_cnt != 0) begin
		/* crc16 CCITT */
		crc16 <= { crc16[14:0], 1'b0 };
		crc16[12] <= crc16[11] ^ crc16_in[7] ^ crc16[15];
		crc16[5] <= crc16[4] ^ crc16_in[7] ^ crc16[15];
		crc16[0] <= crc16_in[7] ^ crc16[15];
		crc16_cnt <= crc16_cnt - 1'b1;
		crc16_in <= { crc16_in[6:0], 1'b0 };
	end

	/* read interface */
	if (msg_rd_en && msg_ready) begin
		recv_rptr <= recv_rptr + 1'b1;
	end

	/* reset */
	if (clr)
		recv_wptr <= 0;
end

reg [7:0] send_crc16_in = 0;
reg [3:0] send_crc16_cnt = 0;
reg [15:0] send_crc16 = 0;
reg [7:0] send_crc1 = 0;
reg [7:0] send_crc2 = 0;
reg [7:0] send_len = 0;
reg [3:0] send_seq = 0;
localparam SST_IDLE = 3'd0;
localparam SST_SOF = 3'd1;	/* start of frame (== len byte) */
localparam SST_SEQ = 3'd2;	/* read sequence byte */
localparam SST_DATA = 3'd3;	/* read data */
localparam SST_CRC1 = 3'd4;	/* read crc1 */
localparam SST_CRC2 = 3'd5;	/* read crc2 */
localparam SST_EOF = 3'd6;	/* read sync byte (end of frame) */
reg [2:0] send_state = RST_SOF;
/*
 * send state machine
 */
always @(posedge clk) begin
	if (!tx_transmitting && !tx_en) begin
		if (send_state == SST_IDLE && send_acknak) begin
			send_acknak <= 0;
			send_state <= SST_SOF;
			send_len <= 8'd5;
			send_seq <= recv_next_seq;
		end else if (send_state == SST_SOF) begin
			send_state <= SST_SEQ;
			tx_data <= send_len;
			tx_en <= 1;
			send_crc16_in <= send_len;
			send_crc16_cnt <= 8;
		end else if (send_state == SST_SEQ) begin
			tx_data <= { 4'b001, send_seq };
			tx_en <= 1;
			send_crc16_in <= { 4'b001, send_seq };
			send_crc16_cnt <= 8;
			if (send_len != 5) begin
				send_state <= SST_DATA;
			end else begin
				send_state <= SST_CRC1;
			end
		end else if (send_state == SST_DATA) begin
			tx_data <= { 4'b0001, send_seq };
			tx_en <= 1;
			send_crc16_in <= { 4'b0001, send_seq };
			send_crc16_cnt <= 8;
			send_state <= SST_CRC1;
		end else if (send_state == SST_CRC1) begin
			tx_data <= send_crc16[15:8];
			tx_en <= 1;
			send_state <= SST_CRC2;
		end else if (send_state == SST_CRC2) begin
			tx_data <= send_crc16[7:0];
			tx_en <= 1;
			send_state <= SST_EOF;
		end else if (send_state == SST_EOF) begin
			tx_data <= RECV_EOF_CHAR;
			tx_en <= 1;
			send_state <= SST_IDLE;
		end
	end
	if (tx_en) begin
		tx_en <= 0;
	end
	if (send_crc16_cnt != 0) begin
		/* crc16 CCITT */
		send_crc16 <= { send_crc16[14:0], 1'b0 };
		send_crc16[12] <= send_crc16[11] ^ send_crc16_in[7] ^
			send_crc16[15];
		send_crc16[5] <= send_crc16[4] ^ send_crc16_in[7] ^
			send_crc16[15];
		send_crc16[0] <= send_crc16_in[7] ^ send_crc16[15];
		send_crc16_cnt <= send_crc16_cnt - 1'b1;
		send_crc16_in <= { send_crc16_in[6:0], 1'b0 };
	end
end

endmodule
