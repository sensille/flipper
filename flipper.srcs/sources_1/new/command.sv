`timescale 1ns / 1ps
`default_nettype none

module command #(
	parameter RING_BITS = 8,
	parameter LEN_BITS = 8,
	parameter LEN_FIFO_BITS = 7,
	parameter CMD_ACKNAK = 8'hff,
	parameter MOVE_COUNT = 512,
	parameter NGPIO = 4,
	parameter NPWM = 6,
	parameter NSTEPDIR = 6
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
	input wire [63:0] clock,

	/*
	 * I/O
	 */
	output wire [NGPIO-1:0] gpio,
	output wire [NPWM-1:0] pwm,
	output wire [NSTEPDIR-1:0] step,
	output wire [NSTEPDIR-1:0] dir,

	/*
	 * debug
	 */
	output wire [63:0] debug
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
localparam CMD_SET_DIGITAL_OUT = 11;
localparam CMD_ALLOCATE_OIDS = 12;
localparam CMD_CONFIG_SOFT_PWM_OUT = 13;
localparam CMD_SCHEDULE_SOFT_PWM_OUT = 14;
localparam CMD_CONFIG_STEPPER = 15;
localparam CMD_CONFIG_ENDSTOP = 16;
localparam CMD_ENDSTOP_HOME = 17;
localparam CMD_ENDSTOP_QUERY_STATE = 18;
localparam RSP_ENDSTOP_STATE = 19;
localparam CMD_ENDSTOP_SET_STEPPER = 20;
localparam CMD_QUEUE_STEP = 21;
localparam CMD_STEPPER_GET_POSITION = 22;
localparam RSP_STEPPER_POSITION = 23;
localparam CMD_SET_NEXT_STEP_DIR = 24;
localparam CMD_RESET_STEP_CLOCK = 25;
/*
      "allocate_oids count=%c":12,
      "config_soft_pwm_out oid=%c pin=%u cycle_ticks=%u value=%c default_value=%c max_duration=%u":13,
      "schedule_soft_pwm_out oid=%c clock=%u on_ticks=%u":14,

      "config_stepper oid=%c step_pin=%c dir_pin=%c min_stop_interval=%u invert_step=%c":15,
      "config_endstop oid=%c pin=%c pull_up=%c stepper_count=%c":16,
      "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c":17,
      "endstop_query_state oid=%c":18,
      "endstop_set_stepper oid=%c pos=%c stepper_oid=%c":20,
      "queue_step oid=%c interval=%u count=%hu add=%hi":21,
      "stepper_get_position oid=%c":22,
      "set_next_step_dir oid=%c dir=%c":24,
      "reset_step_clock oid=%c clock=%u":25

      "endstop_state oid=%c homing=%c pin_value=%c":19,
      "stepper_position oid=%c pos=%i":23

*/

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
localparam MST_SHUTDOWN = 14;

reg [3:0] msg_state = 0;
reg [7:0] msg_cmd = 0;	/* TODO: correct size */
localparam MAX_ARGS = 8;
localparam ARGS_BITS = $clog2(MAX_ARGS);
reg [31:0] args[MAX_ARGS];
reg [ARGS_BITS-1:0] curr_arg = 0;
reg [ARGS_BITS-1:0] nargs = 0;


localparam IDENTIFY_LEN = 551;
localparam IDENTIFY_LEN_BITS = $clog2(IDENTIFY_LEN);

/*
 * pin names
 * Pins are organized in blocks of 16
 * 2 blocks are reserved for GPIO
 */
localparam PIN_GPIO_BASE = 0;
localparam PIN_GPIO_BITS = 5;
localparam PIN_GPIO_NUM = NGPIO;
localparam PIN_SCK = 32;
localparam PIN_SDI = 33;
localparam PIN_SDO = 34;
localparam PIN_CS_BASE = 48;
localparam PIN_CS_BITS = 4;
localparam PIN_CS_NUM = 2;
localparam PIN_DIR_BASE = 64;
localparam PIN_STEP_BASE = 80;
localparam PIN_STEPDIR_BITS = 4;
localparam PIN_STEPDIR_NUM = NSTEPDIR;
localparam PIN_PWM_BASE = 96;
localparam PIN_PWM_BITS = 4;
localparam PIN_PWM_NUM = NPWM;
localparam PIN_ENDSTOP_BASE = 112;
localparam PIN_ENDSTOP_BITS = 4;
localparam PIN_ENDSTOP_NUM = 8;
localparam PIN_MAX = 128;
localparam PIN_BITS = $clog2(PIN_MAX);
localparam OID_MAX = PIN_MAX;
localparam OID_BITS = $clog2(OID_MAX);
localparam ALLOCATED_BIT = OID_BITS;

/*
 * oid mapping. for now, we just store the pin name there,
 * as all pins have dedicated functions
 * Additionally, we have one 'allocated' bit
 */
reg [ALLOCATED_BIT:0] oids[OID_MAX];

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

/* gpio */
reg [NGPIO-1:0] gpio_out = 0;
assign gpio = gpio_out;

/* pwm */
localparam PWM_BITS = 26;
reg [PWM_BITS-1:0] pwm_cycle_ticks[NPWM];
reg [PWM_BITS-1:0] pwm_on_ticks[NPWM];
reg [PWM_BITS-1:0] pwm_next_on_ticks[NPWM];
reg [31:0] pwm_next_clock[NPWM];
reg pwm_scheduled[NPWM];
reg pwm_default_value[NPWM];
reg [31:0] pwm_max_duration[NPWM];
reg [31:0] pwm_duration[NPWM];
integer i;
initial begin
	for (i = 0; i < NPWM; i = i + 1) begin
		pwm_cycle_ticks[i] = 0;
		pwm_on_ticks[i] = 0;
		pwm_next_on_ticks[i] = 0;
		pwm_next_clock[i] = 0;
		pwm_default_value[i] = 1'b0;
		pwm_max_duration[i] = 0;
		pwm_duration[i] = 0;
		pwm_scheduled[i] = 0;
	end
end
genvar pwm_gi;
generate
	for (pwm_gi = 0; pwm_gi < NPWM; pwm_gi = pwm_gi + 1) begin : genpwm
		pwm #(
			.PWM_BITS(PWM_BITS)
		) u_pwm (
			.clk(clk),
			.cycle_ticks(pwm_cycle_ticks[pwm_gi]),
			.on_ticks(pwm_on_ticks[pwm_gi]),
			.out(pwm[pwm_gi])
		);
	end
endgenerate

/*
 * stepdir
 */
reg [31:0] stepdir_[NPWM];
reg [31:0] step_min_stop_interval[NSTEPDIR];
reg invert_step[NSTEPDIR];
/*
 * queue is 72 bits wide:
 * <move type:3> <dir:1> <interval:22> <count:26> <add:20>
 */
localparam MOVE_TYPE_KLIPPER = 3'b000;
localparam MOVE_TYPE_BITS = 3;
localparam STEP_INTERVAL_BITS = 22;
localparam STEP_COUNT_BITS = 26;
localparam STEP_ADD_BITS = 20;
reg [71:0] stepper_queue_wr_data;
reg [NSTEPDIR-1:0] stepper_queue_wr_en = 0;
reg [NSTEPDIR-1:0] step_next_dir = 0;
wire [NSTEPDIR-1:0] step_error;
reg [NSTEPDIR-1:0] step_start = 0;
genvar stepdir_gi;
wire [31:0] step_position[NSTEPDIR];
generate
	for (stepdir_gi = 0; stepdir_gi < NPWM; stepdir_gi = stepdir_gi + 1) begin : genstepdir
		stepdir #(
			.MOVE_TYPE_KLIPPER(MOVE_TYPE_KLIPPER),
			.MOVE_TYPE_BITS(MOVE_TYPE_BITS),
			.STEP_INTERVAL_BITS(STEP_INTERVAL_BITS),
			.STEP_COUNT_BITS(STEP_COUNT_BITS),
			.STEP_ADD_BITS(STEP_ADD_BITS),
			.MOVE_COUNT(MOVE_COUNT)
		) u_stepdir (
			.clk(clk),
			.queue_wr_data(stepper_queue_wr_data),
			.queue_wr_en(stepper_queue_wr_en[stepdir_gi]),
			.start(step_start[stepdir_gi]),
			.reset(0),
			.step(step[stepdir_gi]),
			.dir(dir[stepdir_gi]),
			.error(step_error[stepdir_gi]),
			.position(step_position[stepdir_gi])
		);
	end
endgenerate

localparam MAX_PARAMS = 8;
localparam PARAM_BITS = $clog2(MAX_PARAMS);
reg [31:0] params [MAX_PARAMS];
reg [PARAM_BITS-1:0] nparams;
reg [PARAM_BITS-1:0] curr_param;
reg [34:0] rcv_param;
reg [2:0] curr_cnt;	/* counter for VLQ */
reg [7:0] rsp_len;
reg [PIN_BITS-1:0] oid2pin = 0;

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
	step_start <= 0;
	/*
	 * for convenience resolve the oid to pin lookup.
	 * it's registered, so oids can be implemented in BRAM
	 */
	oid2pin <= oids[args[0][OID_BITS-1:0]];
	/*
	 * ------------------------------------
	 * stage 1, parse arguments, decode VLQ
	 * ------------------------------------
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
			end else if (msg_data == CMD_SET_DIGITAL_OUT) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 2;
			end else if (msg_data == CMD_ALLOCATE_OIDS) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 1;
			end else if (msg_data == CMD_CONFIG_SOFT_PWM_OUT) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 6;
			end else if (msg_data == CMD_SCHEDULE_SOFT_PWM_OUT) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 3;
			end else if (msg_data == CMD_CONFIG_STEPPER) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 5;
			end else if (msg_data == CMD_CONFIG_ENDSTOP) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 4;
			end else if (msg_data == CMD_ENDSTOP_HOME) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 6;
			end else if (msg_data == CMD_ENDSTOP_QUERY_STATE) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 1;
			end else if (msg_data == CMD_ENDSTOP_SET_STEPPER) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 3;
			end else if (msg_data == CMD_QUEUE_STEP) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 4;
			end else if (msg_data == CMD_STEPPER_GET_POSITION) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 1;
			end else if (msg_data == CMD_SET_NEXT_STEP_DIR) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 2;
			end else if (msg_data == CMD_RESET_STEP_CLOCK) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 2;
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
	 * -------------------------
	 * stage 2, dispatch message
	 * -------------------------
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
		end else if (msg_cmd == CMD_SET_DIGITAL_OUT) begin
			if (args[0][31:31-PIN_GPIO_BITS+1] != PIN_GPIO_BASE) begin
				msg_state <= MST_SHUTDOWN;
			end else if (args[0][PIN_GPIO_BITS-1:0] >= PIN_GPIO_NUM) begin
				msg_state <= MST_SHUTDOWN;
			end else begin
				gpio_out[args[0][PIN_GPIO_BITS-1:0]] <= args[1][0];
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_ALLOCATE_OIDS) begin
			/* we have a static array of oids. check only, do nothing */
			if (args[0] > OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else begin
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_CONFIG_SOFT_PWM_OUT) begin
			/*
			 * config_soft_pwm_out oid=%c pin=%u cycle_ticks=%u
			 *                     value=%c default_value=%c max_duration=%u
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][31:PIN_PWM_BITS] != PIN_PWM_BASE[31:PIN_PWM_BITS]) begin
				/* no PWM pin */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][PIN_PWM_BITS-1:0] >= PIN_PWM_NUM) begin
				/* pin out of range */
				msg_state <= MST_SHUTDOWN;
			end else begin
				oids[args[0][OID_BITS-1:0]] <= { 1'b1, args[1][OID_BITS-1:0] };
				pwm_cycle_ticks[args[1][PIN_PWM_BITS-1:0]] <= args[2];
				pwm_on_ticks[args[1][PIN_PWM_BITS-1:0]] <= { PWM_BITS { args[3][0] } };
				pwm_default_value[args[1][PIN_PWM_BITS-1:0]] <= args[4][0];
				pwm_max_duration[args[1][PIN_PWM_BITS-1:0]] <= args[5];
				pwm_duration[args[1][PIN_PWM_BITS-1:0]] <= args[5];
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_SCHEDULE_SOFT_PWM_OUT) begin
			/*
			 * schedule_soft_pwm_out oid=%c clock=%u on_ticks=%u
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_PWM_BITS] !=
			             PIN_PWM_BASE[OID_BITS-1:PIN_PWM_BITS]) begin
				msg_state <= MST_SHUTDOWN;
			end else begin
				pwm_next_clock[oid2pin[PIN_PWM_BITS-1:0]] <= args[1];
				pwm_next_on_ticks[oid2pin[PIN_PWM_BITS-1:0]] <= args[2];
				pwm_scheduled[oid2pin[PIN_PWM_BITS-1:0]] <= 1;
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_CONFIG_STEPPER) begin
			/*
			 * config_stepper oid=%c step_pin=%c dir_pin=%c
			 *                min_stop_interval=%u invert_step=%c
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][31:PIN_STEPDIR_BITS] != PIN_STEP_BASE[31:PIN_STEPDIR_BITS]) begin
				/* no step pin */
				msg_state <= MST_SHUTDOWN;
			end else if (args[2][31:PIN_STEPDIR_BITS] != PIN_DIR_BASE[31:PIN_STEPDIR_BITS]) begin
				/* no dir pin */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][PIN_STEPDIR_BITS-1:0] != args[2][PIN_STEPDIR_BITS-1:0]) begin
				/* step and dir pins don't match */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][PIN_STEPDIR_BITS-1:0] >= PIN_STEPDIR_NUM) begin
				/* step and dir out of range */
				msg_state <= MST_SHUTDOWN;
			end else begin
				oids[args[0][OID_BITS-1:0]] <= { 1'b1, args[1][OID_BITS-1:0] };
				step_min_stop_interval[args[1][31:0]] <= args[3];
				invert_step[args[1][31:0]] <= args[4][0];
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_CONFIG_ENDSTOP) begin
			/*
			 * config_endstop oid=%c pin=%c pull_up=%c stepper_count=%c
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][31:PIN_ENDSTOP_BITS] != PIN_ENDSTOP_BASE[31:PIN_ENDSTOP_BITS]) begin
				/* no endstop pin */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][PIN_STEPDIR_BITS-1:0] >= PIN_ENDSTOP_NUM) begin
				/* pin out of range */
				msg_state <= MST_SHUTDOWN;
			end else begin
				oids[args[0][OID_BITS-1:0]] <= { 1'b1, args[1][OID_BITS-1:0] };
				/* ignore parameters */
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_QUEUE_STEP) begin
			/*
			 * queue_step oid=%c interval=%u count=%hu add=%hi
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_STEPDIR_BITS] !=
			             PIN_STEP_BASE[OID_BITS-1:PIN_STEPDIR_BITS]) begin
				/* oid is not for stepdir */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][31:STEP_INTERVAL_BITS] != 0 ||
			             args[2][31:STEP_COUNT_BITS] != 0 ||
			             (args[3][31:STEP_ADD_BITS] != 0 &&
			              args[3][31:STEP_ADD_BITS] != { (32-STEP_ADD_BITS) { 1'b1 }})) begin
				/* parameters out of range */
				msg_state <= MST_SHUTDOWN;
			end else begin
				/* queue parameters to respective stepper queue */
				stepper_queue_wr_data <= {
					MOVE_TYPE_KLIPPER,
					step_next_dir[oid2pin[PIN_STEPDIR_BITS-1:0]],
					args[1][STEP_INTERVAL_BITS-1:0],
					args[2][STEP_COUNT_BITS-1:0],
					args[3][STEP_ADD_BITS-1:0]
				};
				stepper_queue_wr_en[oid2pin[PIN_STEPDIR_BITS-1:0]] <= 1;
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_RESET_STEP_CLOCK) begin
			/*
			 * reset_step_clock oid=%c clock=%u
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_STEPDIR_BITS] !=
			             PIN_STEP_BASE[OID_BITS-1:PIN_STEPDIR_BITS]) begin
				/* oid is not for stepdir */
				msg_state <= MST_SHUTDOWN;
			end begin
				/* TODO */
				step_start[oid2pin[OID_BITS-1:PIN_STEPDIR_BITS]] <= 1;
				msg_state <= MST_IDLE;
			end
		end else begin
			msg_state <= MST_IDLE;
		end
	/*
	 * ---------------------------------
	 * stage 3, continuation of dispatch
	 * ---------------------------------
	 */
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
	end else if (msg_state == MST_SHUTDOWN) begin
		/* for now, just stay here */
	/*
	 * ---------------------------------
	 * stage 4, encode and send response
	 * ---------------------------------
	 */
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
	end
	/*
	 * pwm duration safety feature. Must be before state
	 * machine because pwm_duration is set on load.
	 */
	for (int i = 0; i < NPWM; i = i + 1) begin
		if (pwm_duration[i] == 1) begin
			pwm_on_ticks[i] <= { PWM_BITS { pwm_default_value[i] } };
		end else if (pwm_duration[i] != 0) begin
			pwm_duration[i] <= pwm_duration[i] - 1;
		end
	end
	/*
	 * loading of pwm_on_ticks, schedule
	 */
	for (int i = 0; i < NPWM; i = i + 1) begin
		if (pwm_scheduled[i] && pwm_next_clock[i] == clock[31:0]) begin
			pwm_on_ticks[i] <= pwm_next_on_ticks[i];
			pwm_duration[i] <= pwm_max_duration[i];
			pwm_scheduled[i] <= 0;
		end
	end
end

assign debug[3:0] = msg_state;
assign debug[11:4] = msg_cmd;
assign debug[37:12] = pwm_cycle_ticks[0];
assign debug[63:38] = pwm_on_ticks[0];

endmodule
