`default_nettype none

module top (
    input CLK, BTN_N, BTN1, BTN2, BTN3,
	output LED1, LED2, LED3, LED4, LED5,
	output P1A1, P1A2, P1A3, P1A4, P1A7, P1A8, P1A9, P1A10,
	output P1B1, P1B2, P1B3, P1B4, P1B7, P1B8, P1B9, P1B10,
);
    // instruction memory
    wire [31:0] pc;
    reg [31:0] imem[0:255];
    initial $readmemh("riscvtest.txt", imem);
    wire [31:0] instr = imem[pc[31:2]];

    // spram as data memory
    wire mem_write;
    wire [31:0] mem_addr, mem_wdata, ram_rdata;
    spram128kB memory (
		.clk(CLK),
		.wen(mem_addr < 4 * 32768 ? {4{mem_write}} : 4'b0), // WORDS: 32k = 128kB / 4
		.addr(mem_addr[16:2]),
		.wdata(mem_wdata),
		.rdata(ram_rdata)
	);

    // memory mapped io regs
    wire [4:0] ioregs;
    wire isIO = mem_addr[17];
    always @(posedge CLK)
        if (~BTN_N)
            ioregs <= 0;
        else if (mem_write)
            ioregs <= mem_wdata[4:0];

    assign {LED5, LED4, LED3, LED2, LED1} = ioregs;
    wire [31:0] mem_rdata = isIO ? ioregs : ram_rdata;

    pipelined core (
        .clk(CLK), .reset(~BTN_N),
        .pc(pc), .instr(instr),
        .mem_write(mem_write), .mem_addr(mem_addr),
        .mem_rdata(mem_rdata), .mem_wdata(mem_wdata)
    );
	// assign { P1A10, P1A9, P1A8, P1A7, P1A4, P1A3, P1A2, P1A1 } = sseg1;
	// assign { P1B10, P1B9, P1B8, P1B7, P1B4, P1B3, P1B2, P1B1 } = sseg2;

    // wire [7:0] sseg1, sseg2;

	// Assign 7 segment control line bus to Pmod pins

	// 7 segment display control
	// seven_seg_ctrl sseg_ctrl1 (
	// 	.CLK(CLK),
	// 	.din(ioregs[7:0]),
        // .dout(sseg1)
	// );
	// seven_seg_ctrl sseg_ctrl2 (
	// 	.CLK(CLK),
	// 	.din(pc[15:8]),
	// 	.dout(sseg2)
	// );
endmodule

// Seven segment controller
// Switches quickly between the two parts of the display
// to create the illusion of both halfs being illuminated
// at the same time.
module seven_seg_ctrl (
	input CLK,
	input [7:0] din,
	output reg [7:0] dout
);
	wire [6:0] lsb_digit;
	wire [6:0] msb_digit;

	seven_seg_hex msb_nibble (
		.din(din[7:4]),
		.dout(msb_digit)
	);

	seven_seg_hex lsb_nibble (
		.din(din[3:0]),
		.dout(lsb_digit)
	);

	reg [9:0] clkdiv = 0;
	reg clkdiv_pulse = 0;
	reg msb_not_lsb = 0;

	always @(posedge CLK) begin
		clkdiv <= clkdiv + 1;
		clkdiv_pulse <= &clkdiv;
		msb_not_lsb <= msb_not_lsb ^ clkdiv_pulse;

		if (clkdiv_pulse) begin
			if (msb_not_lsb) begin
				dout[6:0] <= ~msb_digit;
				dout[7] <= 0;
			end else begin
				dout[6:0] <= ~lsb_digit;
				dout[7] <= 1;
			end
		end
	end
endmodule

// Convert 4bit numbers to 7 segments
module seven_seg_hex (
	input [3:0] din,
	output reg [6:0] dout
);
	always @*
		case (din)
			4'h0: dout = 7'b 0111111;
			4'h1: dout = 7'b 0000110;
			4'h2: dout = 7'b 1011011;
			4'h3: dout = 7'b 1001111;
			4'h4: dout = 7'b 1100110;
			4'h5: dout = 7'b 1101101;
			4'h6: dout = 7'b 1111101;
			4'h7: dout = 7'b 0000111;
			4'h8: dout = 7'b 1111111;
			4'h9: dout = 7'b 1101111;
			4'hA: dout = 7'b 1110111;
			4'hB: dout = 7'b 1111100;
			4'hC: dout = 7'b 0111001;
			4'hD: dout = 7'b 1011110;
			4'hE: dout = 7'b 1111001;
			4'hF: dout = 7'b 1110001;
			default: dout = 7'b 1000000;
		endcase
endmodule
