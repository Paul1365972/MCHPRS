module uart_receiver #(
    parameter CLOCKS_PER_BIT = 100
) (
    input wire clock,
    input wire serial,
    output reg [7:0] data = 0,
    output reg ready = 0,
    output reg framing_error = 0
);
    (* ASYNC_REG = "TRUE" *) reg synchronize_first = 1;
    (* ASYNC_REG = "TRUE" *) reg synchronize_second = 1;
    localparam COUNTER_WIDTH = CLOCKS_PER_BIT > 1 ? $clog2(CLOCKS_PER_BIT) : 1;
    reg [COUNTER_WIDTH-1:0] clocks_left = 0;
    reg [3:0] position = 0;
    reg [7:0] receiving = 0;
    reg active = 0;

    always @(posedge clock) begin
        synchronize_first <= serial;
        synchronize_second <= synchronize_first;
        ready <= 0;
        framing_error <= 0;
        if (!active) begin
            if (!synchronize_second) begin
                active <= 1;
                clocks_left <= (CLOCKS_PER_BIT > 1 ? CLOCKS_PER_BIT / 2 : 1) - 1;
                position <= 0;
            end
        end else if (clocks_left != 0) begin
            clocks_left <= clocks_left - 1;
        end else begin
            clocks_left <= CLOCKS_PER_BIT - 1;
            if (position == 0) begin
                if (synchronize_second)
                    active <= 0;
                else
                    position <= 1;
            end else if (position <= 8) begin
                receiving[position - 1] <= synchronize_second;
                position <= position + 1;
            end else begin
                active <= 0;
                if (synchronize_second) begin
                    data <= receiving;
                    ready <= 1;
                end else begin
                    framing_error <= 1;
                end
            end
        end
    end
endmodule
