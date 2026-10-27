// It samples each bit in its middle and listens again from the middle of the stop bit,
// so it follows a sender that is slightly faster.
module uart_rx #(
    parameter int CLOCKS_PER_BIT = 25
) (
    input logic clock,
    input logic line,
    output logic valid = 0,
    output logic [7:0] data = 0
);
    localparam int CLOCK_BITS = $clog2(CLOCKS_PER_BIT);

    logic [1:0] line_crossing = '1;
    logic level;
    logic receiving = 0;
    logic [CLOCK_BITS - 1:0] clocks_left = 0;
    logic [3:0] samples = 0;
    logic [7:0] received = 0;

    assign level = line_crossing[1];

    always_ff @(posedge clock) begin
        line_crossing <= {line_crossing[0], line};
        valid <= 0;
        if (!receiving) begin
            receiving <= !level;
            clocks_left <= CLOCK_BITS'(CLOCKS_PER_BIT / 2);
            samples <= 0;
        end else if (clocks_left != 0) begin
            clocks_left <= clocks_left - 1'b1;
        end else begin
            clocks_left <= CLOCK_BITS'(CLOCKS_PER_BIT - 1);
            samples <= samples + 1'b1;
            received <= {level, received[7:1]};
            if (samples == 0 && level) receiving <= 0;
            if (samples == 9) begin
                receiving <= 0;
                valid <= level;
                data <= received;
            end
        end
    end
endmodule
