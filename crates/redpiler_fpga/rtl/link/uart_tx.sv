module uart_tx #(
    parameter int CLOCKS_PER_BIT = 25
) (
    input logic clock,
    input logic valid,
    input logic [7:0] data,
    output logic ready,
    output logic line = 1
);
    localparam int CLOCK_BITS = $clog2(CLOCKS_PER_BIT);
    localparam logic [CLOCK_BITS - 1:0] BIT_END = CLOCK_BITS'(CLOCKS_PER_BIT - 1);

    logic [8:0] following = '1;
    logic [3:0] bits_left = 0;
    logic [CLOCK_BITS - 1:0] clocks_left = 0;

    assign ready = bits_left == 0 || (bits_left == 1 && clocks_left == 0);

    always_ff @(posedge clock) begin
        if (ready && valid) begin
            line <= 0;
            following <= {1'b1, data};
            bits_left <= 10;
            clocks_left <= BIT_END;
        end else if (clocks_left != 0) begin
            clocks_left <= clocks_left - 1'b1;
        end else if (bits_left != 0) begin
            line <= following[0];
            following <= {1'b1, following[8:1]};
            bits_left <= bits_left - 1'b1;
            clocks_left <= BIT_END;
        end
    end
endmodule
