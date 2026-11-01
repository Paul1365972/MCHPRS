module uart_transmitter #(
    parameter CLOCKS_PER_BIT = 100
) (
    input wire clock,
    input wire [7:0] data,
    input wire start,
    output reg serial = 1,
    output reg busy = 0
);
    localparam COUNTER_WIDTH = CLOCKS_PER_BIT > 1 ? $clog2(CLOCKS_PER_BIT) : 1;
    reg [COUNTER_WIDTH-1:0] clocks_left = 0;
    reg [3:0] bits_left = 0;
    reg [8:0] sending = 9'h1ff;

    always @(posedge clock) begin
        if (!busy) begin
            if (start) begin
                sending <= {1'b1, data};
                serial <= 0;
                clocks_left <= CLOCKS_PER_BIT - 1;
                bits_left <= 9;
                busy <= 1;
            end
        end else if (clocks_left != 0) begin
            clocks_left <= clocks_left - 1;
        end else if (bits_left != 0) begin
            sending <= {1'b1, sending[8:1]};
            serial <= sending[0];
            bits_left <= bits_left - 1;
            clocks_left <= CLOCKS_PER_BIT - 1;
        end else begin
            serial <= 1;
            busy <= 0;
        end
    end
endmodule
