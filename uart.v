// default div for icebreaker's 12MHz and 115200 baudrate
module uart #(parameter integer CLK_DIV = 105) (
    input clk,
    input rst,

    output tx,
    input i_valid,
    input [7:0] i_data,
    output tx_done,

    input rx,
    output [7:0] o_data
);
    // count up to CLK_DIV - 1
    reg [$clog2(CLK_DIV) - 1:0] recv_counter;
    reg recv_start;
    reg receiving;
    reg [3:0] recv_bitcnt;
    reg [7:0] recv_buf;
    reg recv_buf_valid;
    assign o_data = recv_buf_valid ? recv_buf : ~0;

    always @(posedge clk) begin
        recv_counter <= recv_counter + 1;
        if (rst) begin
            receiving <= 0;
            recv_start <= 0;
            recv_counter <= 0;
            recv_buf_valid <= 0;
            recv_buf <= 0;
        end else begin
            // start bit
            if (!rx && !receiving) begin
                receiving <= 1;
                recv_bitcnt <= 8;
                recv_counter <= 0;
                recv_start <= 1;
            end

            // synchronise at half the clks per bit
            if (recv_start && 2 * recv_counter == CLK_DIV) begin
                recv_start <= 0;
                recv_counter <= 0;
            end

            // sample
            if (!recv_start && receiving && recv_counter == CLK_DIV - 1) begin
                recv_buf <= {rx, recv_buf[7:1]};
                recv_counter <= 0;
                recv_bitcnt <= recv_bitcnt - 1;
            end

            // stop bit / byte received
            // do we need to wait for the stop bit?
            if (rx &&
                    receiving && !recv_bitcnt) begin
                receiving <= 0;
                recv_buf_valid <= 1;
            end
        end
    end

    reg [9:0] send_buf;
    reg [3:0] send_bitcnt;
    reg [$clog2(CLK_DIV) - 1:0] send_divcnt;
    assign tx = send_buf[0];
    assign tx_done = !send_bitcnt;

    always @(posedge clk) begin
        send_divcnt <= send_divcnt + 1;

        if (rst) begin
            send_buf <= ~0;
            send_bitcnt <= 0;
            send_divcnt <= 0;
        end else
            if (i_valid && tx_done) begin
                send_buf <= {1'b1, i_data, 1'b0};
                send_bitcnt <= 10;
                send_divcnt <= 0;
            end

            if(send_divcnt == CLK_DIV - 1 && send_bitcnt) begin
                send_buf <= {1'b1, send_buf[9:1]};
                send_bitcnt <= send_bitcnt - 1;
                send_divcnt <= 0;
            end
    end
endmodule
