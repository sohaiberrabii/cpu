module test;
    reg reset = 1;
    initial #5 reset <= 0;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rx = 1;
    always #10 rx = ~rx;

    wire tx;

    reg i_valid = 1;
    initial #15 i_valid <= 0;
    reg [7:0] i_data = 8'h41;

    wire o_ready;
    wire [7:0] o_data;

    uart #(.CLK_DIV(2)) uart (
            .clk(clk),
            .rst(reset),

            .rx(rx),
            .tx(tx),

            .i_valid(i_valid),
            .i_data(i_data),
            .o_ready(o_ready),
            .o_data(o_data)
    );

    initial $monitor("tx=%b, send_buf=%b", tx, uart.send_buf);
endmodule
