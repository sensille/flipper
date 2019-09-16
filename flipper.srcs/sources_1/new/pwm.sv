`timescale 1ns / 1ps
`default_nettype none

module pwm #(
	parameter PWM_BITS = 26
) (
	input wire clk,

	/*
	 * config
	 */
	input wire [PWM_BITS-1:0] cycle_ticks,
	input wire [PWM_BITS-1:0] on_ticks,

	/*
	 * output
	 */
	output reg out = 0
);

reg [PWM_BITS-1:0] cycle_cnt = 0;
reg [PWM_BITS-1:0] at_cnt = 0;

always @(posedge clk) begin
	if (cycle_cnt == 0) begin
		cycle_cnt <= cycle_ticks;
		at_cnt <= on_ticks;
		if (on_ticks != 0)
			out <= 1'b1;
	end else begin
		cycle_cnt <= cycle_cnt - 1;
		at_cnt <= at_cnt - 1;
		if (at_cnt == 0) begin
			out <= 1'b0;
		end
	end
end

endmodule
