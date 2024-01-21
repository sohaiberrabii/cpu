module test;
    reg clk = 0;
    always #5 clk = ~clk;

    reg reset = 1;
    initial #5 reset <= 0;

    // memory
    reg [31:0] mem[0:32767];
    initial $readmemh("firmware.hex", mem);
    wire [3:0] mem_write;
    wire [31:0] mem_wdata, mem_addr;
    reg [31:0] mem_rdata;

    always @(posedge clk) begin
        mem_rdata <= mem[mem_addr[16:2]];
        if (mem_write[0]) mem[mem_addr[16:2]][7:0] <= mem_wdata[7:0];
        if (mem_write[1]) mem[mem_addr[16:2]][15:8] <= mem_wdata[15:8];
        if (mem_write[2]) mem[mem_addr[16:2]][23:16] <= mem_wdata[23:16];
        if (mem_write[3]) mem[mem_addr[16:2]][32:24] <= mem_wdata[32:24];
    end

    wire [31:0] pc;
    wire [31:0] instr = mem[pc[31:2]];

    cpu dut (
        .clk(clk), .reset(reset),
        .pc(pc), .instr(instr),
        .mem_write(mem_write), .mem_addr(mem_addr),
        .mem_rdata(mem_rdata), .mem_wdata(mem_wdata)
    );

    // initialize test
    reg [8*32:1] vcdfn;
    initial begin
        if ($value$plusargs("vcd=%s", vcdfn)) $dumpfile(vcdfn);
        $dumpvars(0, test);
    end

    // check test status
    reg [7:0] failed = 0;
    reg [63:0] cycle_count, instr_count;
    always @(posedge clk) begin
        if(|mem_write && mem_addr == 32'h20004)
            $write("%c", mem_wdata[7:0]);

        if (|mem_write && mem_addr == 32'h2000c)
            cycle_count[31:0] <= mem_wdata;
        if (|mem_write && mem_addr == 32'h20010)
            cycle_count[63:32] <= mem_wdata;

        if (|mem_write && mem_addr == 32'h20014)
            instr_count[31:0] <= mem_wdata;
        if (|mem_write && mem_addr == 32'h20018)
            instr_count[63:32] <= mem_wdata;

        // ebreak
        if(instr == 32'h00100073) failed <= failed + 1;
        if(|mem_write && mem_addr == 32'h20008) begin
            if(mem_wdata == 0) begin
                if (failed > 0)
                    $display("Failed %d tests", failed);
                else
                    $display("Tests passed");
            end else
                $write(": %3d", mem_wdata);

            $display("Executed %12d instructions in %12d cycles", instr_count, cycle_count);
            $finish;
        end
    end
endmodule

