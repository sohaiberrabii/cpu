module spram128kB (
	input clk,
	input [3:0] wen,
	input [14:0] addr,
	input [31:0] wdata,
	output [31:0] rdata
);
	wire [31:0] rdata0, rdata1;
	wire cs0 = !addr[14];
    wire cs1 = addr[14];

	assign rdata = addr[14] ? rdata1 : rdata0;

	SB_SPRAM256KA ram00 (
		.ADDRESS(addr[13:0]),
		.DATAIN(wdata[15:0]),
		.MASKWREN({wen[1], wen[1], wen[0], wen[0]}),
		.WREN(wen[1]|wen[0]),
		.CHIPSELECT(cs0),
		.CLOCK(clk),
        .STANDBY(1'b0),
		.SLEEP(1'b0),
		.POWEROFF(1'b1),
		.DATAOUT(rdata0[15:0])
	);

	SB_SPRAM256KA ram01 (
		.ADDRESS(addr[13:0]),
		.DATAIN(wdata[31:16]),
		.MASKWREN({wen[3], wen[3], wen[2], wen[2]}),
		.WREN(wen[3] | wen[2]),
		.CHIPSELECT(cs0),
		.CLOCK(clk),
        .STANDBY(1'b0),
		.SLEEP(1'b0),
		.POWEROFF(1'b1),
		.DATAOUT(rdata0[31:16])
	);

	SB_SPRAM256KA ram10 (
		.ADDRESS(addr[13:0]),
		.DATAIN(wdata[15:0]),
		.MASKWREN({wen[1], wen[1], wen[0], wen[0]}),
		.WREN(wen[1]|wen[0]),
		.CHIPSELECT(cs1),
		.CLOCK(clk),
        .STANDBY(1'b0),
		.SLEEP(1'b0),
		.POWEROFF(1'b1),
		.DATAOUT(rdata1[15:0])
	);

	SB_SPRAM256KA ram11 (
		.ADDRESS(addr[13:0]),
		.DATAIN(wdata[31:16]),
		.MASKWREN({wen[3], wen[3], wen[2], wen[2]}),
		.WREN(wen[3]|wen[2]),
		.CHIPSELECT(cs1),
		.CLOCK(clk),
        .STANDBY(1'b0),
		.SLEEP(1'b0),
		.POWEROFF(1'b1),
		.DATAOUT(rdata1[31:16])
	);
endmodule
