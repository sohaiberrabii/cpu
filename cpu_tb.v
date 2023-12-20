module test;
    wire [31:0] mem_wdata, mem_addr;
    wire mem_write;
    reg clk, reset;

    // instruction memory
    wire [31:0] pc;
    reg [31:0] imem[0:255];
    initial $readmemh("riscvtest.txt", imem);
    wire [31:0] instr = imem[pc[31:2]];

    // data memory
    wire [31:0] mem_rdata;
    spram128kB memory (
		.clk(clk),
		.wen({4{mem_write}}),
		.addr(mem_addr[16:2]),
		.wdata(mem_wdata),
		.rdata(mem_rdata)
	);

    pipelined dut (
        .clk(clk), .reset(reset),
        .pc(pc), .instr(instr),
        .mem_write(mem_write), .mem_addr(mem_addr),
        .mem_rdata(mem_rdata), .mem_wdata(mem_wdata)
    );

    // initialize test
    reg [8*32:1] vcdfn;
    initial begin
        if ($value$plusargs("vcd=%s", vcdfn)) $dumpfile(vcdfn);
        $dumpvars;
        /* reset <= 1; # 22; reset <= 0; */
        reset <= 1; #22 reset <= 0;
    end

    // generate clock to sequence tests
    always begin
        clk <= 1; # 5; clk <= 0; # 5;
    end

    // check results
    always @(negedge clk) begin
        if(mem_write) begin
            if(mem_addr === 100 & mem_wdata === 25) begin
                $display("Simulation succeeded");
                $finish;
            end else if (mem_addr !== 96) begin
                $display("Simulation failed");
                $stop;
            end
        end
    end
endmodule

