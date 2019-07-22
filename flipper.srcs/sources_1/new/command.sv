`timescale 1ns / 1ps
`default_nettype none

module command #(
	parameter RING_BITS = 8,
	parameter LEN_BITS = 8,
	parameter LEN_FIFO_BITS = 7,
	parameter CMD_ACKNAK = 8'hff
) (
	input wire clk,

	/*
	 * receive side
	 */
	input wire [7:0] msg_data,
	input wire msg_ready,
	output reg msg_rd_en = 0,

	/*
	 * send side
	 */
	output reg send_fifo_wr_en = 0,
	output reg [LEN_BITS-1:0] send_fifo_data = 0,
	input wire send_fifo_full,

	/* ring buffer input */
	output reg [7:0] send_ring_data = 0,
	output reg send_ring_wr_en = 0,
	input wire send_ring_full
);

localparam RSP_IDENTIFY = 0;
localparam CMD_IDENTIFY = 1;
localparam MST_IDLE = 0;
localparam MST_PARSE_ARG_START = 1;
localparam MST_PARSE_ARG_CONT = 2;
localparam MST_DISPATCH = 3;
localparam MST_PARAM = 4;
localparam MST_PARAM_SKIP = 5;
localparam MST_PARAM_SEND = 6;
localparam MST_IDENTIFY_SEND = 7;
reg [2:0] msg_state = 0;
reg [7:0] msg_cmd = 0;	/* TODO: correct size */
localparam MAX_ARGS = 8;
reg [31:0] args[MAX_ARGS];
reg [2:0] arg_cnt = 0;

localparam IDENTIFY_LEN = 500;
localparam IDENTIFY_LEN_BITS = $clog2(IDENTIFY_LEN);

reg [7:0] identify_mem[0:499];
initial begin
	$readmemh("identify.mem", identify_mem);
end

localparam MAX_PARAMS = 8;
localparam MAX_PARAM_BITS = $clog2(MAX_PARAMS);
reg [31:0] params [MAX_PARAMS];
reg [MAX_PARAM_BITS-1:0] nparams;
reg [34:0] curr_param;
reg [2:0] curr_cnt;	/* counter for VLQ */
reg [7:0] rsp_len;
always @(posedge clk) begin
	reg _arg_end;
	if (msg_rd_en) begin
		msg_rd_en <= 0;
	end
	if (send_ring_wr_en) begin
		send_ring_wr_en <= 0;
	end
	if (send_fifo_wr_en) begin
		send_fifo_wr_en <= 0;
	end
	if (msg_ready && !msg_rd_en) begin
		_arg_end = 0;
		msg_rd_en <= 1;
		if (msg_state == MST_IDLE) begin
			msg_cmd <= msg_data;
			if (msg_data == CMD_ACKNAK) begin
				send_fifo_data <= 0;
				send_fifo_wr_en <= 1;
				/* state stays at MST_IDLE */
			end else if (msg_data == CMD_IDENTIFY) begin
				msg_state <= MST_PARSE_ARG_START;
				arg_cnt <= 1;
			end
		end else if (msg_state == MST_PARSE_ARG_START) begin
			args[arg_cnt] <= msg_data[6:0];
			if (msg_data[7]) begin
				msg_state <= MST_PARSE_ARG_CONT;
			end else begin
				_arg_end = 1;
			end
		end else if (msg_state == MST_PARSE_ARG_CONT) begin
			args[arg_cnt] <= { args[arg_cnt][24:0], msg_data[6:0] };
			if (!msg_data[7]) begin
				_arg_end = 1;
			end
		end else begin
			/* we're in some of the states below */
			msg_rd_en <= 0;	/* we're in some of the states below */
		end
		if (_arg_end) begin
			arg_cnt <= arg_cnt - 1;
			if (arg_cnt == 0) begin
				msg_state <= MST_DISPATCH;
			end else begin
				msg_state <= MST_PARSE_ARG_START;
			end
		end
	end
	if (msg_state == MST_DISPATCH) begin
		if (msg_cmd == CMD_IDENTIFY) begin
			/*
			 * determine length to send
			 */
			if (args[1] >= IDENTIFY_LEN) begin
				params[0] <= 0;
			end else if (args[1] + args[0] > IDENTIFY_LEN) begin
				params[0] <= IDENTIFY_LEN - args[1];
			end else begin
				params[0] <= args[0];
			end
			nparams <= 0;
			msg_state <= MST_PARAM;
			send_ring_data <= RSP_IDENTIFY;
			send_ring_wr_en <= 1;
			rsp_len <= 1;
		end else begin
			msg_state <= MST_IDLE;
		end
	end else if (msg_state == MST_PARAM) begin
		/*
		 * encode VLQ param
		 */
		/*
		 * < 00000060 && >= ffffffe0 length 1
		 * < 00003000 && >= fffff000 length 2
		 * < 00180000 && >= fff80000 length 3
		 * < 0c000000 && >= fc000000 length 4
		 * else length 5
		 */
		curr_cnt <= 4;
		/* extend by 3 bits, 32+3 == 5 * 7 */
		curr_param <= { params[nparams][31], params[nparams][31],
				params[nparams][31], params[nparams] };
		msg_state <= MST_PARAM_SKIP;
	end else if (msg_state == MST_PARAM_SKIP) begin
		if (curr_cnt != 0 &&
		    (curr_param[34:26] == 9'b111111111 ||
		     curr_param[34:26] <  9'b000000011)) begin
			curr_cnt <= curr_cnt - 1;
			curr_param <= { curr_param[27:0], 7'b0 };
		end else begin
			msg_state <= MST_PARAM_SEND;
		end
	end else if (msg_state == MST_PARAM_SEND) begin
		if (curr_cnt == 0) begin
			if (nparams != 0) begin
				nparams <= nparams - 1;
				msg_state <= MST_PARAM_SKIP;
			end else if (msg_cmd == CMD_IDENTIFY) begin
				msg_state <= MST_IDENTIFY_SEND;
			end else begin
				send_fifo_data <= rsp_len;
				send_fifo_wr_en <= 1;
				msg_state <= MST_IDLE;
			end
			send_ring_data[7] <= 1'b0;
		end else begin
			send_ring_data[7] <= 1'b1;
		end
		send_ring_data[6:0] <= curr_param[34:28];
		send_ring_wr_en <= 1;
		rsp_len <= rsp_len + 1;
		curr_cnt <= curr_cnt - 1;
		curr_param <= { curr_param[27:0], 7'b0 };
	end else if (msg_state == MST_IDENTIFY_SEND) begin
		if (params[0] == 0) begin
			send_fifo_data <= rsp_len;
			send_fifo_wr_en <= 1;
			msg_state <= MST_IDLE;
		end else begin
			send_ring_data <= identify_mem[args[1]];
			args[1] <= args[1] + 1;
			send_ring_wr_en <= 1;
			rsp_len <= rsp_len + 1;
			params[0] <= params[0] - 1;
		end
	end
end

endmodule
