//`timescale 10ns / 1ns

module test;
    wire [31:0] writedata, dataaddr;
    wire memwrite;
    reg clk, reset;

    // instantiate device to be tested
    top dut(.CLK(clk), .BTN_N(~reset), .memwrite(memwrite), .wdata(writedata), .addr(dataaddr));

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

