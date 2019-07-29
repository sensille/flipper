`timescale 1ns / 1ps
`default_nettype none

module command #(
	parameter RING_BITS = 8,
	parameter LEN_BITS = 8,
	parameter LEN_FIFO_BITS = 7,
	parameter CMD_ACKNAK = 8'hff,
	parameter MOVE_COUNT = 64
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
	input wire send_ring_full,

	/* global clock */
	input wire [63:0] clock
);

localparam RSP_IDENTIFY = 0;
localparam CMD_IDENTIFY = 1;
localparam CMD_GET_CONFIG = 2;
localparam RSP_GET_CONFIG = 3;
localparam CMD_FINALIZE_CONFIG = 4;
localparam CMD_GET_CLOCK = 5;
localparam RSP_CLOCK = 6;
localparam CMD_GET_UPTIME = 7;
localparam RSP_UPTIME = 8;
localparam MST_IDLE = 0;
localparam MST_PARSE_ARG_START = 1;
localparam MST_PARSE_ARG_CONT = 2;
localparam MST_DISPATCH = 3;
localparam MST_PARAM = 4;
localparam MST_PARAM_SKIP = 5;
localparam MST_PARAM_SEND = 6;
localparam MST_IDENTIFY_1 = 7;
localparam MST_IDENTIFY_SEND = 8;
localparam MST_GET_CONFIG_1 = 9;
localparam MST_GET_CONFIG_2 = 10;
localparam MST_GET_CONFIG_3 = 11;
localparam MST_GET_CONFIG_4 = 12;
localparam MST_GET_UPTIME_1 = 13;
reg [3:0] msg_state = 0;
reg [7:0] msg_cmd = 0;	/* TODO: correct size */
localparam MAX_ARGS = 8;
localparam ARGS_BITS = $clog2(MAX_ARGS);
reg [31:0] args[0:MAX_ARGS];
reg [ARGS_BITS-1:0] curr_arg = 0;
reg [ARGS_BITS-1:0] nargs = 0;

localparam IDENTIFY_LEN = 364;
localparam IDENTIFY_LEN_BITS = $clog2(IDENTIFY_LEN);

/*
 * pin names
 * Pins are organized in blocks of 16
 * 2 blocks are reserved for GPIO
 */
localparam PIN_GPIO_BASE = 0;
localparam PIN_GPIO_BITS = 5;
localparam PIN_GPIO_NUM = 1;
localparam PIN_SCK = 32;
localparam PIN_SDI = 33;
localparam PIN_SDO = 34;
localparam PIN_CS_BASE = 48;
localparam PIN_CS_BITS = 4;
localparam PIN_CS_NUM = 2;
localparam PIN_DIR_BASE = 64;
localparam PIN_DIR_BITS = 4;
localparam PIN_DIR_NUM = 6;
localparam PIN_STEP_BASE = 80;
localparam PIN_STEP_BITS = 4;
localparam PIN_STEP_NUM = 6;
localparam PIN_PWM_BASE = 96;
localparam PIN_PWM_BITS = 4;
localparam PIN_PWM_NUM = 6;
localparam PIN_ES_BASE = 112;
localparam PIN_ES_BITS = 4;
localparam PIN_ES_NUM = 8;

reg [7:0] identify_mem[0:IDENTIFY_LEN-1];
initial begin
	$readmemh("identify.mem", identify_mem);
end

/*
 * global state
 */
reg [31:0] config_crc = 0;
reg is_finalized = 0;
reg is_shutdown = 0;

localparam MAX_PARAMS = 8;
localparam PARAM_BITS = $clog2(MAX_PARAMS);
reg [31:0] params [MAX_PARAMS];
reg [PARAM_BITS-1:0] nparams;
reg [PARAM_BITS-1:0] curr_param;
reg [34:0] rcv_param;
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
	/*
	 * stage 1, parse arguments, decode VLQ
	 */
	if (msg_ready && !msg_rd_en) begin
		_arg_end = 0;
		msg_rd_en <= 1;
		if (msg_state == MST_IDLE) begin
			msg_cmd <= msg_data;
			curr_arg <= 0;
			curr_param <= 0;
			if (msg_data == CMD_ACKNAK) begin
				send_fifo_data <= 0;
				send_fifo_wr_en <= 1;
				/* state stays at MST_IDLE */
			end else if (msg_data == CMD_IDENTIFY) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 2;
			end else if (msg_data == CMD_GET_CONFIG) begin
				msg_state <= MST_DISPATCH;
			end else if (msg_data == CMD_FINALIZE_CONFIG) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 1;
			end else if (msg_data == CMD_GET_UPTIME) begin
				msg_state <= MST_DISPATCH;
			end else if (msg_data == CMD_GET_CLOCK) begin
				msg_state <= MST_DISPATCH;
			end
		end else if (msg_state == MST_PARSE_ARG_START) begin
			args[curr_arg] <= msg_data[6:0];
			if (msg_data[7]) begin
				msg_state <= MST_PARSE_ARG_CONT;
			end else begin
				_arg_end = 1;
			end
		end else if (msg_state == MST_PARSE_ARG_CONT) begin
			args[curr_arg] <= { args[curr_arg][24:0], msg_data[6:0] };
			if (!msg_data[7]) begin
				_arg_end = 1;
			end
		end else begin
			/* we're in some of the states below */
			msg_rd_en <= 0;	/* we're in some of the states below */
		end
		if (_arg_end) begin
			curr_arg <= curr_arg + 1;
			if (curr_arg + 1 == nargs) begin
				msg_state <= MST_DISPATCH;
			end else begin
				msg_state <= MST_PARSE_ARG_START;
			end
		end
	end
	/*
	 * stage 2, dispatch message
	 */
	if (msg_state == MST_DISPATCH) begin
		if (msg_cmd == CMD_IDENTIFY) begin
			/* echo offset */
			params[0] <= args[0];
			msg_state <= MST_IDENTIFY_1;
		end else if (msg_cmd == CMD_GET_CONFIG) begin
			msg_state <= MST_GET_CONFIG_1;
		end else if (msg_cmd == CMD_FINALIZE_CONFIG) begin
			/* TODO: check if already finalized */
			config_crc <= args[0];
			is_finalized <= 1;
			msg_state <= MST_GET_CONFIG_1;
		end else if (msg_cmd == CMD_GET_UPTIME) begin
			send_ring_data <= RSP_UPTIME;
			send_ring_wr_en <= 1;
			rsp_len <= 1;
			params[0] <= clock[63:32];
			msg_state <= MST_GET_UPTIME_1;
		end else if (msg_cmd == CMD_GET_CLOCK) begin
			send_ring_data <= RSP_CLOCK;
			send_ring_wr_en <= 1;
			rsp_len <= 1;
			params[0] <= clock[31:0];
			nparams <= 1;
			msg_state <= MST_PARAM;
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
		rcv_param <= { params[curr_param][31], params[curr_param][31],
				params[curr_param][31], params[curr_param] };
		msg_state <= MST_PARAM_SKIP;
	end else if (msg_state == MST_PARAM_SKIP) begin
		if (curr_cnt != 0 &&
		    (rcv_param[34:26] == 9'b111111111 ||
		     rcv_param[34:26] <  9'b000000011)) begin
			curr_cnt <= curr_cnt - 1;
			rcv_param <= { rcv_param[27:0], 7'b0 };
		end else begin
			msg_state <= MST_PARAM_SEND;
		end
	end else if (msg_state == MST_PARAM_SEND) begin
		if (curr_cnt == 0) begin
			if (curr_param + 1 != nparams) begin
				curr_param <= curr_param + 1;
				msg_state <= MST_PARAM;
			end else if (msg_cmd == CMD_IDENTIFY) begin
				msg_state <= MST_IDENTIFY_SEND;
			end else begin
				send_fifo_data <= rsp_len + 1;
				send_fifo_wr_en <= 1;
				msg_state <= MST_IDLE;
			end
			send_ring_data[7] <= 1'b0;
		end else begin
			send_ring_data[7] <= 1'b1;
		end
		send_ring_data[6:0] <= rcv_param[34:28];
		send_ring_wr_en <= 1;
		rsp_len <= rsp_len + 1;
		curr_cnt <= curr_cnt - 1;
		rcv_param <= { rcv_param[27:0], 7'b0 };
	/*
	 * continuation of identify
	 */
	end else if (msg_state == MST_IDENTIFY_1) begin
		/*
		 * determine length to send
		 */
		if (args[0] >= IDENTIFY_LEN) begin
			params[1] <= 0;
		end else if (args[0] + args[1] > IDENTIFY_LEN) begin
			params[1] <= IDENTIFY_LEN - args[0];
		end else begin
			params[1] <= args[1];
		end
		nparams <= 2;
		msg_state <= MST_PARAM;
		send_ring_data <= RSP_IDENTIFY;
		send_ring_wr_en <= 1;
		rsp_len <= 1;
	end else if (msg_state == MST_IDENTIFY_SEND) begin
		if (params[1] == 0) begin
			send_fifo_data <= rsp_len;
			send_fifo_wr_en <= 1;
			msg_state <= MST_IDLE;
		end else begin
			send_ring_data <= identify_mem[args[0]];
			args[0] <= args[0] + 1;
			send_ring_wr_en <= 1;
			rsp_len <= rsp_len + 1;
			params[1] <= params[1] - 1;
		end
	/*
	 * continuation of get_config, also in response to
	 * finalize_config
	 */
	end else if (msg_state == MST_GET_CONFIG_1) begin
		send_ring_data <= RSP_GET_CONFIG;
		send_ring_wr_en <= 1;
		rsp_len <= 1;
		params[0] <= is_finalized;
		msg_state <= MST_GET_CONFIG_2;
	end else if (msg_state == MST_GET_CONFIG_2) begin
		params[1] <= config_crc;
		msg_state <= MST_GET_CONFIG_3;
	end else if (msg_state == MST_GET_CONFIG_3) begin
		params[2] <= MOVE_COUNT;
		msg_state <= MST_GET_CONFIG_4;
	end else if (msg_state == MST_GET_CONFIG_4) begin
		params[3] <= is_shutdown;
		nparams <= 4;
		msg_state <= MST_PARAM;
	end else if (msg_state == MST_GET_UPTIME_1) begin
		params[1] <= clock[31:0];
		nparams <= 2;
		msg_state <= MST_PARAM;
	end
end

endmodule
