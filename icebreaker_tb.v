module test;
    reg clk = 0;
    always #5 clk = ~clk;

    reg reset = 1;
    initial #5 reset <= 0;

    // initialize test
    reg [8*32:1] vcdfn;
    initial begin
        $readmemh("firmware.hex", dut.imem);
        if ($value$plusargs("vcd=%s", vcdfn)) $dumpfile(vcdfn);
        $dumpvars(0, test);

		repeat (6) begin
			repeat (50000) @(posedge clk);
			$display("+50000 cycles");
		end
    end

	wire led1, led2, led3, led4, led5;
	wire [4:0] leds = {led5, led4, led3, led2, led1};
	always @(leds) begin
		#1 $display("%b", leds);
	end

	icebreaker #(.MEMWORDS(256)) dut (
		.CLK(clk),
        .BTN_N(~reset),
		.LED1(led1),
		.LED2(led2),
		.LED3(led3),
		.LED4(led4),
		.LED5(led5)
	);

    // check results
    always @(negedge clk) begin
        if (dut.mem_write & dut.mem_addr == 32'h20004)
        if (dut.mem_wdata == 0) begin
            $display("Simulation succeeded");
            $finish;
        end else begin
            $display("Failed test %d", dut.mem_wdata);
            $stop;
        end
    end
endmodule
