module uart #(parameter integer CLK_DIV = 2) (
    input clk,
    input rst,

    output tx,
    input rx,

    input i_valid,
    input [7:0] i_data,
    output o_ready,
    output [7:0] o_data
);

    reg [$clog2(CLK_DIV):0] send_divcnt;
    reg [3:0] send_bitcnt;

    reg [9:0] recv_buf;
    reg [9:0] send_buf;

    assign tx = send_buf[0];
    assign o_ready = !send_bitcnt;

    always @(posedge clk) begin
        send_divcnt <= send_divcnt + 1;

        if (rst) begin
            send_buf <= ~0;
            send_bitcnt <= 0;
            send_divcnt <= 0;
        end else
            if (i_valid && o_ready) begin
                send_buf <= {1'b1, i_data, 1'b0};
                send_bitcnt <= 10;
                send_divcnt <= 0;
            end else
            if(send_divcnt == CLK_DIV && send_bitcnt) begin
                send_buf <= {1'b1, send_buf[9:1]};
                send_bitcnt <= send_bitcnt - 1;
                send_divcnt <= 0;
            end
    end
endmodule
