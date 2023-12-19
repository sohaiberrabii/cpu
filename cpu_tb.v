module test;
    wire [31:0] writedata, dataaddr;
    wire memwrite;
    reg clk, reset;

    // instruction memory
    wire [31:0] pc;
    reg [31:0] imem[0:255];
    initial $readmemh("riscvtest.txt", imem);
    wire [31:0] instr = imem[pc[31:2]];

    // data memory
    wire [31:0] mem_rdata;
    reg [31:0] dmem [255:0];
    always @(posedge clk)
        if (memwrite)
            dmem[dataaddr[31:2]] <= writedata;
    assign mem_rdata = dmem[dataaddr[31:2]];

    pipelined dut (
        .clk(clk), .reset(reset),
        .pc(pc), .instr(instr),
        .mem_write(memwrite), .mem_addr(dataaddr),
        .mem_rdata(mem_rdata), .mem_wdata(writedata)
    );

    // initialize test
    reg [8*32:1] vcdfn;
    initial begin
        if ($value$plusargs("vcd=%s", vcdfn)) $dumpfile(vcdfn);
        $dumpvars(0, dut);
        /* reset <= 1; # 22; reset <= 0; */
        reset <= 1; #22 reset <= 0;
    end

    // generate clock to sequence tests
    always begin
        clk <= 1; # 5; clk <= 0; # 5;
    end

    // check results
    always @(negedge clk) begin
        if(memwrite) begin
            if(dataaddr === 100 & writedata === 25) begin
                $display("Simulation succeeded");
                $finish;
            end else if (dataaddr !== 96) begin
                $display("Simulation failed");
                $stop;
            end
        end
    end
endmodule

