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
	parameter NSTEPDIR = 6,
	parameter NENDSTOP = 8,
	parameter NSPI = 2,
	parameter NCS = 5
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
	input wire [NENDSTOP-1:0] endstop,
	output reg [NSPI-1:0] msck = 0,
	output reg [NSPI-1:0] mosi = 0,
	input wire [NSPI-1:0] miso,
	output reg [NCS-1:0] mcsn = { NCS { 1'b1 }},

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
localparam CMD_SPI_SET_BUS = 26;
localparam CMD_CONFIG_SPI = 27;
localparam CMD_SPI_SEND = 28;
localparam CMD_SPI_TRANSFER = 29;
localparam RSP_SPI_TRANSFER = 30;

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

      "spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u":26,
      "config_spi oid=%c pin=%u":27,
      "spi_send oid=%c data=%*s":29,
      "spi_transfer oid=%c data=%*s":30
      "spi_transfer_response oid=%c response=%*s":31
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
localparam MST_GET_STEPPER_POSITION_1 = 14;
localparam MST_GET_ENDSTOP_STATE_1 = 15;
localparam MST_GET_ENDSTOP_STATE_2 = 16;
localparam MST_GET_ENDSTOP_STATE_3 = 17;
localparam MST_ENDSTOP_SET_STEPPER_1 = 18;
localparam MST_ENDSTOP_SET_STEPPER_2 = 19;
localparam MST_STRING_START = 20;
localparam MST_STRING_ARG = 21;
localparam MST_SPI_TRANSFER_1 = 22;
localparam MST_SPI_TRANSFER_SEND = 23;
localparam MST_SPI_TRANSFER_CLK_LO = 24;
localparam MST_SPI_TRANSFER_CLK_HI = 25;
localparam MST_SPI_TRANSFER_END = 26;
localparam MST_PARAM_END = 27;
localparam MST_SHUTDOWN = 31;

reg [4:0] msg_state = 0;
reg [7:0] msg_cmd = 0;	/* TODO: correct size */
localparam MAX_ARGS = 8;
localparam ARGS_BITS = $clog2(MAX_ARGS);
reg [31:0] args[MAX_ARGS];
reg [ARGS_BITS-1:0] curr_arg = 0;
reg [ARGS_BITS-1:0] nargs = 0;


localparam IDENTIFY_LEN = 792;
localparam IDENTIFY_LEN_BITS = $clog2(IDENTIFY_LEN);

/*
 * pin names
 * Pins are organized in blocks of 16
 * 2 blocks are reserved for GPIO
 */
localparam PIN_GPIO_BASE = 0;
localparam PIN_GPIO_BITS = 5;
localparam PIN_GPIO_NUM = NGPIO;
localparam PIN_SPI_BASE = 32;
localparam PIN_SPI_BITS = 4;
localparam PIN_SPI_NUM = NSPI;
localparam PIN_CS_BASE = 48;
localparam PIN_CS_BITS = 4;
localparam PIN_CS_NUM = NCS;
localparam PIN_DIR_BASE = 64;
localparam PIN_STEP_BASE = 80;
localparam PIN_STEPDIR_BITS = 4;
localparam PIN_STEPDIR_NUM = NSTEPDIR;
localparam PIN_PWM_BASE = 96;
localparam PIN_PWM_BITS = 4;
localparam PIN_PWM_NUM = NPWM;
localparam PIN_ENDSTOP_BASE = 112;
localparam PIN_ENDSTOP_BITS = 4;
localparam PIN_ENDSTOP_NUM = NENDSTOP;
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

/* shutdown reasons */
localparam SR_BAD_PIN = 1;
localparam SR_PIN_OUT_OF_RANGE = 2;

/*
 * global state
 */
reg [31:0] config_crc = 0;
reg is_finalized = 0;
reg is_shutdown = 0;
reg [7:0] shutdown_reason = 0;

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
reg [71:0] step_queue_wr_data;
reg [NSTEPDIR-1:0] step_queue_wr_en = 0;
wire [NSTEPDIR-1:0] step_queue_empty;
wire [NSTEPDIR-1:0] step_running;
reg [NSTEPDIR-1:0] step_next_dir = 0;
reg [NSTEPDIR-1:0] step_start = 0;
reg [NSTEPDIR-1:0] step_reset = 0;
reg [NSTEPDIR-1:0] step_start_pending = 0;
reg [31:0] step_start_clock [NSTEPDIR-1:0];
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
			.queue_wr_data(step_queue_wr_data),
			.queue_wr_en(step_queue_wr_en[stepdir_gi]),
			.queue_empty(step_queue_empty[stepdir_gi]),
			.running(step_running[stepdir_gi]),
			.start(step_start[stepdir_gi]),
			.reset(step_reset[stepdir_gi]),
			.step(step[stepdir_gi]),
			.dir(dir[stepdir_gi]),
			.position(step_position[stepdir_gi])
		);
	end
endgenerate

/*
 * endstops
 */
reg [NENDSTOP-1:0] endstop_homing = 0;
reg [NSTEPDIR-1:0] endstop_stepper[NENDSTOP-1:0];
reg [31:0] endstop_clock[NENDSTOP-1:0];
reg [15:0] endstop_sample_ticks[NENDSTOP-1:0];	/* hardcoded max */
reg [5:0] endstop_sample_count[NENDSTOP-1:0];
reg [15:0] endstop_tick_cnt[NENDSTOP-1:0];
reg [5:0] endstop_sample_cnt[NENDSTOP-1:0];
reg endstop_pin_value[NENDSTOP];
reg [NSTEPDIR-1:0] endstop_step_reset[NENDSTOP-1:0];
reg [NENDSTOP-1:0] endstop_send_state = 0;
reg [OID_BITS-1:0] endstop_oid [NENDSTOP-1:0];
localparam ES_IDLE = 0;
localparam ES_WAIT_FOR_CLOCK = 1;
localparam ES_REST = 2;
localparam ES_SAMPLE = 3;
reg [1:0] endstop_state[NENDSTOP-1:0];
initial begin
	for (int i = 0; i < NENDSTOP; i = i + 1) begin
		endstop_stepper[i] <= 0;
		endstop_step_reset[i] <= { NSTEPDIR { 1'b0 }};
	end
end
always @(*) begin
	step_reset = 0;
	for (int i = 0; i < NENDSTOP; i = i + 1) begin
		if (endstop_step_reset[i]) begin
			step_reset |= endstop_stepper[i];
		end
	end
end

/*
 * spi
 * spi parameters are stored per cs
 */
reg [31:0] spi_rate[NCS];
reg [PIN_SPI_BITS-1:0] spi_bus[NCS];
reg [30:0] spi_cnt = 0;
reg [2:0] spi_bits = 0;
reg [5:0] spi_bytes = 0;
reg [7:0] spi_data_out = 0;
reg [7:0] spi_data_in = 0;

localparam MAX_PARAMS = 8;
localparam PARAM_BITS = $clog2(MAX_PARAMS);
reg [31:0] params [MAX_PARAMS];
reg [PARAM_BITS-1:0] nparams;
reg [PARAM_BITS-1:0] curr_param;
reg [34:0] rcv_param;
reg [2:0] curr_cnt;	/* counter for VLQ */
reg [7:0] rsp_len;
reg [PIN_BITS-1:0] oid2pin = 0;
reg write_oid = 0;
reg oid2pin_arg2 = 0;
reg [PIN_ENDSTOP_BITS-1:0] endstop_tmp = 0; /* tmp storage for endstop_set_stepper */
reg [PIN_ENDSTOP_BITS-1:0] _endstop_send_ix;
reg [OID_BITS-1:0] _endstop_send_oid;
reg string_arg = 0;
/* assume max string is 64 */
reg [5:0] str_pos = 0;
reg [5:0] str_len = 0;
reg [7:0] str_buf[64];

/* shorthands */
wire [PIN_CS_BITS-1:0] spi_cs_r = oid2pin[PIN_CS_BITS-1:0];
wire [PIN_SPI_BITS-1:0] spi_bus_r = spi_bus[spi_cs_r];

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
	step_queue_wr_en <= 0;
	/*
	 * channel all read/write to oids array here, so it
	 * can be properly inferred as RAM
	 */
	if (write_oid) begin
		oids[args[0][OID_BITS-1:0]] <= { 1'b1, args[1][OID_BITS-1:0] };
		write_oid <= 0;
	end else if (oid2pin_arg2) begin
		oid2pin <= oids[args[2][OID_BITS-1:0]];
		oid2pin_arg2 <= 0;
	end else begin
		oid2pin <= oids[args[0][OID_BITS-1:0]];
	end
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
			end else if (msg_data == CMD_SPI_SET_BUS) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 4;
			end else if (msg_data == CMD_CONFIG_SPI) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 2;
			end else if (msg_data == CMD_SPI_SEND) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 2;
				string_arg <= 1;
			end else if (msg_data == CMD_SPI_TRANSFER) begin
				msg_state <= MST_PARSE_ARG_START;
				nargs <= 2;
				string_arg <= 1;
			end
		end else if (msg_state == MST_PARSE_ARG_START) begin
			args[curr_arg] <= msg_data[6:0];
			if (msg_data[6:5] == 2'b11) begin
				/* negative value */
				args[curr_arg][31:7] <= 25'b1111111111111111111111111;
			end
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
		end else if (msg_state == MST_STRING_START) begin
			string_arg <= 0;
			str_pos <= 0;
			str_len <= args[curr_arg - 1];
			msg_state <= MST_STRING_ARG;
			msg_rd_en <= 0;	/* no read in this clock */
		end else if (msg_state == MST_STRING_ARG) begin
			str_buf[str_pos] <= msg_data;
			str_pos <= str_pos + 1;
			if (str_len == str_pos + 1) begin
				msg_state <= MST_DISPATCH;
			end
		end else begin
			/* we're in some of the states below */
			msg_rd_en <= 0;	/* we're in some of the states below */
		end
		if (_arg_end) begin
			curr_arg <= curr_arg + 1;
			if (curr_arg + 1 == nargs) begin
				if (string_arg) begin
					msg_state <= MST_STRING_START;
				end else begin
					msg_state <= MST_DISPATCH;
				end
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
				write_oid <= 1;
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
				write_oid <= 1;
				/* min_stop_interval ignored as it is not deemed useful */
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
			end else if (args[3] >= NSTEPDIR) begin
				/* can't stop more steppers than we have */
				msg_state <= MST_SHUTDOWN;
			end else begin
				write_oid <= 1;
				/* save oid for endstop_state message */
				endstop_oid[args[1][PIN_ENDSTOP_BITS-1:0]] <= args[0];
				/* ignore parameters */
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_QUEUE_STEP) begin
			/*
			 * queue_step oid=%c interval=%u count=%hu add=%hi
			 */
			/* TODO: shutdown if queue is empty and no reset_step pending */
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
			end else if (!step_running[oid2pin[PIN_STEPDIR_BITS-1:0]] &&
			             !step_start_pending[oid2pin[PIN_STEPDIR_BITS-1:0]]) begin
				/* "step in the future", queue underrun */
				msg_state <= MST_SHUTDOWN;
			end else begin
				/* queue parameters to respective stepper queue */
				step_queue_wr_data <= {
					MOVE_TYPE_KLIPPER,
					step_next_dir[oid2pin[PIN_STEPDIR_BITS-1:0]],
					args[1][STEP_INTERVAL_BITS-1:0],
					args[2][STEP_COUNT_BITS-1:0],
					args[3][STEP_ADD_BITS-1:0]
				};
				step_queue_wr_en[oid2pin[PIN_STEPDIR_BITS-1:0]] <= 1;
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
			end else if (!step_queue_empty[oid2pin[PIN_STEPDIR_BITS-1:0]] ||
			             step_running[oid2pin[PIN_STEPDIR_BITS-1:0]]) begin
				/* queue is not empty */
				msg_state <= MST_SHUTDOWN;
			end begin
				step_start_clock[oid2pin[PIN_STEPDIR_BITS-1:0]] <= args[1];
				step_start_pending[oid2pin[PIN_STEPDIR_BITS-1:0]] <= 1;
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_SET_NEXT_STEP_DIR) begin
			/*
			 * set_next_step_dir oid=%c dir=%c
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_STEPDIR_BITS] !=
			             PIN_STEP_BASE[OID_BITS-1:PIN_STEPDIR_BITS]) begin
				/* oid is not for stepdir */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][31:1] != 0) begin
				/* argument out of range */
				msg_state <= MST_SHUTDOWN;
			end begin
				step_next_dir[oid2pin[PIN_STEPDIR_BITS-1:0]] <= args[1][0];
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_STEPPER_GET_POSITION) begin
			/*
			 * stepper_get_position oid=%c
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_STEPDIR_BITS] !=
			             PIN_STEP_BASE[OID_BITS-1:PIN_STEPDIR_BITS]) begin
				/* oid is not for stepdir */
				msg_state <= MST_SHUTDOWN;
			end begin
				send_ring_data <= RSP_STEPPER_POSITION;
				send_ring_wr_en <= 1;
				rsp_len <= 1;
				params[0] <= args[0];
				msg_state <= MST_GET_STEPPER_POSITION_1;
			end
		end else if (msg_cmd == CMD_ENDSTOP_QUERY_STATE) begin
			/*
			 * endstop_query_state oid=%c
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_ENDSTOP_BITS] !=
			             PIN_ENDSTOP_BASE[OID_BITS-1:PIN_ENDSTOP_BITS]) begin
				/* oid is not for endstop */
				msg_state <= MST_SHUTDOWN;
			end begin
				_endstop_send_oid <= args[0];
				_endstop_send_ix <= oid2pin[PIN_ENDSTOP_BITS-1:0];
				msg_state <= MST_GET_ENDSTOP_STATE_1;
			end
		end else if (msg_cmd == CMD_ENDSTOP_SET_STEPPER) begin
			/*
			 * endstop_set_stepper oid=%c pos=%c stepper_oid=%c
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_ENDSTOP_BITS] !=
			             PIN_ENDSTOP_BASE[OID_BITS-1:PIN_ENDSTOP_BITS]) begin
				/* oid is not for endstop */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1] >= NSTEPDIR) begin
				/* pos out of range */
				msg_state <= MST_SHUTDOWN;
			end else if (args[2] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end begin
				/* resolve stepper oid */
				oid2pin_arg2 <= 1;
				endstop_tmp <= oid2pin[PIN_ENDSTOP_BITS-1:0];
				msg_state <= MST_ENDSTOP_SET_STEPPER_1;
			end
		end else if (msg_cmd == CMD_ENDSTOP_HOME) begin
			/*
			 * endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_ENDSTOP_BITS] !=
			             PIN_ENDSTOP_BASE[OID_BITS-1:PIN_ENDSTOP_BITS]) begin
				/* oid is not for endstop */
				msg_state <= MST_SHUTDOWN;
			end if (args[2] > 65535) begin
				/* max sample_ticks exceeped */
				msg_state <= MST_SHUTDOWN;
			end if (args[3] > 31) begin
				/* max sample_count exceeped */
				msg_state <= MST_SHUTDOWN;
			end if (args[5] > 1) begin
				/* invalid pin value */
				msg_state <= MST_SHUTDOWN;
			end if (args[3] == 0) begin
				/* cancel homing operation */
				endstop_homing[oid2pin[PIN_ENDSTOP_BITS-1:0]] <= 0;
			end else begin
				endstop_clock[oid2pin[PIN_ENDSTOP_BITS-1:0]] <= args[1];
				endstop_sample_ticks[oid2pin[PIN_ENDSTOP_BITS-1:0]] <= args[2];
				endstop_sample_count[oid2pin[PIN_ENDSTOP_BITS-1:0]] <= args[3];
				/* rest ticks are ignored */
				endstop_pin_value[oid2pin[PIN_ENDSTOP_BITS-1:0]] <= args[5];
				endstop_homing[oid2pin[PIN_ENDSTOP_BITS-1:0]] <= 1;
				endstop_state[oid2pin[PIN_ENDSTOP_BITS-1:0]] <= ES_WAIT_FOR_CLOCK;
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_CONFIG_SPI) begin
			/*
			 * config_spi oid=%c pin=%u
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][31:PIN_CS_BITS] != PIN_CS_BASE[31:PIN_CS_BITS]) begin
				/* no cs pin */
				shutdown_reason <= SR_BAD_PIN;
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][PIN_CS_BITS-1:0] >= PIN_CS_NUM) begin
				/* cs pin out of range */
				shutdown_reason <= SR_PIN_OUT_OF_RANGE;
				msg_state <= MST_SHUTDOWN;
			end else begin
				write_oid <= 1;
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_SPI_SET_BUS) begin
			/*
			 * spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_CS_BITS] !=
			             PIN_CS_BASE[OID_BITS-1:PIN_CS_BITS]) begin
				/* no cs oid */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][31:PIN_SPI_BITS] != PIN_SPI_BASE[31:PIN_SPI_BITS]) begin
				/* no spi bus */
				msg_state <= MST_SHUTDOWN;
			end else if (args[1][PIN_SPI_BITS-1:0] >= PIN_SPI_NUM) begin
				/* spi bus out of range */
				msg_state <= MST_SHUTDOWN;
			end else if (args[2] != 3) begin
				/* only mode 3 supported */
				msg_state <= MST_SHUTDOWN;
			end else begin
				spi_rate[oid2pin[PIN_CS_BITS-1:0]] <= args[3];
				spi_bus[oid2pin[PIN_CS_BITS-1:0]] <= args[1][PIN_SPI_BITS-1:0];
				msg_state <= MST_IDLE;
			end
		end else if (msg_cmd == CMD_SPI_TRANSFER ||
		             msg_cmd == CMD_SPI_SEND) begin
			/*
			 * spi_transfer oid=%c data=%*s
			 * spi_send oid=%c data=%*s
			 */
			if (args[0] >= OID_MAX) begin
				msg_state <= MST_SHUTDOWN;
			end else if (oid2pin[OID_BITS-1:PIN_CS_BITS] !=
			             PIN_CS_BASE[OID_BITS-1:PIN_CS_BITS]) begin
				/* no cs oid */
				msg_state <= MST_SHUTDOWN;
			end else begin
				msg_state <= MST_SPI_TRANSFER_1;
			end
/*
      "spi_transfer_response oid=%c response=%*s":31
 */
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
	end else if (msg_state == MST_GET_STEPPER_POSITION_1) begin
		params[1] <= step_position[oid2pin[PIN_STEPDIR_BITS-1:0]];
		nparams <= 2;
		msg_state <= MST_PARAM;
	end else if (msg_state == MST_GET_ENDSTOP_STATE_1) begin
		send_ring_data <= RSP_ENDSTOP_STATE;
		send_ring_wr_en <= 1;
		rsp_len <= 1;
		params[0] <= _endstop_send_oid;
		msg_state <= MST_GET_ENDSTOP_STATE_2;
	end else if (msg_state == MST_GET_ENDSTOP_STATE_2) begin
		params[1] <= endstop_homing[_endstop_send_ix];
		msg_state <= MST_GET_ENDSTOP_STATE_3;
	end else if (msg_state == MST_GET_ENDSTOP_STATE_3) begin
		params[2] <= endstop[_endstop_send_ix];
		nparams <= 3;
		msg_state <= MST_PARAM;
	end else if (msg_state == MST_ENDSTOP_SET_STEPPER_1) begin
		/* need one more clock to resolve stepper oid */
		msg_state <= MST_ENDSTOP_SET_STEPPER_2;
	end else if (msg_state == MST_ENDSTOP_SET_STEPPER_2) begin
		if (oid2pin[OID_BITS-1:PIN_STEPDIR_BITS] !=
			     PIN_STEP_BASE[OID_BITS-1:PIN_STEPDIR_BITS]) begin
			/* oid is not for stepdir */
			msg_state <= MST_SHUTDOWN;
		end else begin
			endstop_stepper[endstop_tmp][oid2pin[PIN_STEPDIR_BITS-1:0]] <= 1;
			msg_state <= MST_IDLE;
		end
	end else if (msg_state == MST_SPI_TRANSFER_1) begin
		if (msg_cmd == CMD_SPI_TRANSFER) begin
			/* transfer cmd/oid/len first, then continue to data */
			params[0] <= args[0];	/* echo oid */
			params[1] <= args[1];	/* send len == rcv len*/
			nparams <= 2;
			send_ring_data <= RSP_SPI_TRANSFER;
			send_ring_wr_en <= 1;
			rsp_len <= 1;
			msg_state <= MST_PARAM;
		end else begin
			/* nothing to send, go to data phase directly */
			msg_state <= MST_SPI_TRANSFER_SEND;
		end
		/*
		 * msck needs to be 1 for at least 10ns before CS goes low.
		 * As we run with 20MHz, the condition is met with one cycle
		 */
		msck[spi_bus_r] <= 1;
	end else if (msg_state == MST_SPI_TRANSFER_SEND) begin
		mcsn[spi_cs_r] <= 0;
		spi_cnt <= 0;
		spi_bits <= 0;
		spi_bytes <= 0;
		spi_data_out <= str_buf[0];
		msg_state <= MST_SPI_TRANSFER_CLK_HI;
	end else if (msg_state == MST_SPI_TRANSFER_CLK_HI) begin
		if (spi_cnt + 1 != spi_rate[spi_cs_r][30:1]) begin
			spi_cnt <= spi_cnt + 1;
		end else begin
			spi_cnt <= 0;
			msck[spi_bus_r] <= 0;
			mosi[spi_bus_r] <= spi_data_out[0];
			spi_data_out <= { 1'b0, spi_data_out[7:1] };
			msg_state <= MST_SPI_TRANSFER_CLK_LO;
		end
	end else if (msg_state == MST_SPI_TRANSFER_CLK_LO) begin
		if (spi_cnt + 1 != spi_rate[spi_cs_r][30:1]) begin
			spi_cnt <= spi_cnt + 1;
		end else begin
			spi_cnt <= 0;
			msck[spi_bus_r] <= 1;
			spi_data_in <= { spi_data_in[6:0], miso[spi_bus_r] };
			spi_bits <= spi_bits + 1;
			msg_state <= MST_SPI_TRANSFER_CLK_HI;
			if (spi_bits == 7) begin
				spi_data_out <= str_buf[spi_bytes + 1];
				if (msg_cmd == CMD_SPI_TRANSFER) begin
					send_ring_data <= { spi_data_in[6:0], miso[spi_bus_r] };
					send_ring_wr_en <= 1;
					rsp_len <= rsp_len + 1;
				end
				spi_bytes <= spi_bytes + 1;
				if (spi_bytes + 1 == args[1]) begin
					msg_state <= MST_SPI_TRANSFER_END;
				end
			end
		end
	end else if (msg_state == MST_SPI_TRANSFER_END) begin
		mcsn[spi_cs_r] <= 1;
		if (msg_cmd == CMD_SPI_TRANSFER) begin
			send_fifo_data <= rsp_len;
			send_fifo_wr_en <= 1;
		end
		msg_state <= MST_IDLE;
	end else if (msg_state == MST_SHUTDOWN) begin
		/* for now, just stay here */
		is_shutdown <= 1;
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
			end else if (msg_cmd == CMD_SPI_TRANSFER) begin
				msg_state <= MST_SPI_TRANSFER_SEND;
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
	 * ------------------------------
	 * stage 5, send involuntary data
	 * ------------------------------
	 *
	 * only send when state machine is idle and no msg has
	 * been pulled at stage 1
	 */
	end else if (!msg_ready && msg_state == MST_IDLE) begin
		for (int i = 0; i < NENDSTOP; i = i + 1) begin
			if (endstop_send_state[i]) begin
				endstop_send_state[i] <= 0;
				_endstop_send_oid <= endstop_oid[i];
				_endstop_send_ix <= i;
				msg_state <= MST_GET_ENDSTOP_STATE_1;
				break;
			end
		end
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
	/*
	 * reset_step_clock handling
	 */
	for (int i = 0; i < NSTEPDIR; i = i + 1) begin
		if (step_start_pending[i] && clock == step_start_clock[i]) begin
			step_start_pending[i] <= 0;
			step_start[i] <= 1;
		end
	end
	/*
	 * endstop homing engine
	 */
	for (int i = 0; i < NENDSTOP; i = i + 1) begin
		if (endstop_homing[i]) begin
			if (endstop_state[i] == ES_WAIT_FOR_CLOCK) begin
				if (endstop_clock[i] == clock) begin
					endstop_state[i] <= ES_REST;
				end
			end else if (endstop_state[i] == ES_REST) begin
				if (endstop[i] == endstop_pin_value[i]) begin
					endstop_state[i] <= ES_SAMPLE;
					endstop_tick_cnt[i] <= endstop_sample_ticks[i];
					endstop_sample_cnt[i] <= endstop_sample_count[i];
				end
			end else if (endstop_state[i] == ES_SAMPLE) begin
				if (endstop_tick_cnt[i] == 1) begin
					if (endstop[i] != endstop_pin_value[i]) begin
						endstop_state[i] <= ES_REST;
					end else begin
						if (endstop_sample_cnt[i] == 1) begin
							/* endstop triggered */
							endstop_homing[i] <= 0;
							/* reset stepper */
							endstop_step_reset[i] <= 1;
							/* send endstop_state */
							endstop_send_state[i] <= 1;
						end else begin
							endstop_sample_cnt[i] <= endstop_sample_cnt[i] - 1;
						end
					end
				end else begin
					endstop_tick_cnt[i] <= endstop_tick_cnt[i] - 1;
				end
			end
		end
	end
end

assign debug[3:0] = msg_state;
assign debug[11:4] = msg_cmd;
assign debug[37:12] = pwm_cycle_ticks[0];
assign debug[63:38] = pwm_on_ticks[0];

endmodule
