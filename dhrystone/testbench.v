`timescale 1 ns / 1 ps

module testbench;
    reg clk = 0;
    always #5 clk = ~clk;

    reg reset = 1;
    initial #5 reset <= 0;

    wire [3:0] mem_write;
    wire [31:0] mem_wdata, mem_addr;
    reg [31:0] mem_rdata;

    reg [31:0] mem[0:64*1024-1];
	initial $readmemh("dhry.hex", mem);

    always @(posedge clk) begin
        mem_rdata <= mem[mem_addr[16:2]];
        case (mem_addr)
            32'h1000_0000: begin
                $write("%c", mem_wdata);
                $fflush();
            end
            default: begin
                if (mem_write[0]) mem[mem_addr[17:2]][7:0] <= mem_wdata[7:0];
                if (mem_write[1]) mem[mem_addr[17:2]][15:8] <= mem_wdata[15:8];
                if (mem_write[2]) mem[mem_addr[17:2]][23:16] <= mem_wdata[23:16];
                if (mem_write[3]) mem[mem_addr[17:2]][32:24] <= mem_wdata[32:24];
            end
        endcase
    end

    wire [31:0] pc;
    wire [31:0] instr = mem[pc[31:2]];

    cpu #(.PROGADDR_RESET(32'h10000)) dut (
        .clk(clk), .reset(reset),
        .pc(pc), .instr(instr),
        .mem_write(mem_write), .mem_addr(mem_addr),
        .mem_rdata(mem_rdata), .mem_wdata(mem_wdata)
    );

	initial begin
		$dumpfile("testbench.vcd");
		$dumpvars(0, testbench);
	end

	always @(posedge clk) begin
        // ebreak
        if(instr == 32'h00100073) begin
            @(posedge clk);
            if (!dut.flushe) begin
                repeat (10) @(posedge clk);
                $display("EBREAK");
                $finish;
            end
		end
	end
endmodule
