`timescale 1ns/10ps

module uart_tb;
    parameter clk_period = 100; // 10 MHz at 1ns timescale
    parameter clks_per_bit = $ceil(10_000_000 / 115200);
    parameter bit_duration = $floor(10_000_000 / 115200) * clk_period;

    reg clk = 0;
    always #(clk_period / 2) clk <= !clk;

    reg reset = 1;

    wire tx;
    reg i_valid = 0;
    reg [7:0] i_data = 0;
    wire tx_done;

    reg rx = 1;
    wire [7:0] o_data;

    uart #(.CLK_DIV(clks_per_bit)) uart (
        .clk(clk),
        .rst(reset),

        .tx(tx),
        .i_valid(i_valid),
        .i_data(i_data),
        .tx_done(tx_done),

        .rx(rx),
        .o_data(o_data)
    );

    task UART_WRITE_BYTE(input [7:0] byte);
        integer i;
        begin
            // start bit
            rx <= 1'b0;
            #(bit_duration);

            // data byte
            for (i = 0; i < 8; i = i + 1) begin
                rx <= byte[i];
                #(bit_duration);
            end

            // stop bit
            rx <= 1'b1;
            #(bit_duration);
        end
    endtask

    task UART_READ_BYTE(output [7:0] byte);
        integer i;
        begin
            // start bit
            #(bit_duration);
            if (tx != 0) begin
                $display("no start bit at tx");
                $stop;
            end

            // data byte
            for (i = 0; i < 8; i = i + 1) begin
                #(bit_duration);
                byte[i] <= tx;
            end

            // stop bit
            #(bit_duration);
            if (tx != 1) begin
                $display("no stop bit at tx");
                $stop;
            end
        end
    endtask

    reg [7:0] tx_byte;

    initial begin
        $dumpfile("uart_tb.vcd");
        $dumpvars;

        @(posedge clk);
        reset <= 0;
        @(posedge clk);

        // tx test
        i_valid <= 1;
        i_data <= 8'hAB;
        @(posedge clk);

        i_valid <= 0;
        UART_READ_BYTE(tx_byte);

        if (tx_byte == 8'hAB)
            $display("tx test passed - correct byte sent");
        else
            $display("tx test failed - incorrect byte snt");

        // @(posedge tx_done);

        // rx test
        @(posedge clk);
        UART_WRITE_BYTE(8'h3F);

        if (o_data == 8'h3F)
            $display("rx test passed - correct byte received");
        else
            $display("rx test failed - incorrect byte received");

        $finish;
    end

endmodule
