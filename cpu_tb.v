module test;
    reg clk, reset;

    // memory
    reg [31:0] mem[0:32767];
    initial $readmemh("firmware.hex", mem);

    wire mem_write;
    wire [31:0] mem_wdata, mem_addr;
    reg [31:0] mem_rdata;
    always @(posedge clk)
        if (mem_write)
            mem[mem_addr[16:2]] <= mem_wdata;
        else
            mem_rdata <= mem[mem_addr[16:2]];

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
        /* reset <= 1; # 22; reset <= 0; */
        reset <= 1; #22 reset <= 0;
    end

    // generate clock to sequence tests
    always begin
        clk <= 1; # 5; clk <= 0; # 5;
    end

    // check results
    always @(negedge clk) begin
        if(mem_write & mem_addr == 32'h20008)
            if(mem_wdata == 0) begin
                $display("Simulation succeeded");
                $finish;
            end else begin
                $display("Failed test %d", mem_wdata);
                $stop;
            end
    end
endmodule

