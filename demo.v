`default_nettype none

module top (
    input CLK, BTN_N, BTN1, BTN2, BTN3,
	output LED1, LED2, LED3, LED4, LED5,
	output P1A1, P1A2, P1A3, P1A4, P1A7, P1A8, P1A9, P1A10,
	output P1B1, P1B2, P1B3, P1B4, P1B7, P1B8, P1B9, P1B10,
);
    wire [31:0] pc, pctarget;

    // TODO: separate instruction and data memory from core

    // Useful when 7 segment display is tracking the pc, replace CLK below with clkdiv_pulse
	// Clock divider and pulse registers
	reg [23:0] clkdiv = 0;
	reg clkdiv_pulse = 0;
    always @(posedge CLK) begin
		// Clock divider pulse generator
        // 12MHz
		if (clkdiv == 12000000) begin
			clkdiv <= 0;
			clkdiv_pulse <= 1;
		end else begin
			clkdiv <= clkdiv + 1;
			clkdiv_pulse <= 0;
		end

    end

    pipelined core (.clk(CLK), .reset(~BTN_N), .pc(pc), .pctarget(pctarget));

	assign LED1 = ~BTN_N;

    wire [7:0] sseg1, sseg2;

	// Assign 7 segment control line bus to Pmod pins
	assign { P1A10, P1A9, P1A8, P1A7, P1A4, P1A3, P1A2, P1A1 } = sseg1;
	assign { P1B10, P1B9, P1B8, P1B7, P1B4, P1B3, P1B2, P1B1 } = sseg2;

	// 7 segment display control
	seven_seg_ctrl sseg_ctrl1 (
		.CLK(CLK),
		.din(pc[7:0]),
		.dout(sseg1)
	);
	seven_seg_ctrl sseg_ctrl2 (
		.CLK(CLK),
		.din(pctarget[7:0]),
		.dout(sseg2)
	);
endmodule

