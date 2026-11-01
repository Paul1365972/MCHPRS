module activity #(
    parameter HOLD_CLOCKS = 1
) (
    input wire clock,
    input wire signal,
    output wire active
);
    reg [$clog2(HOLD_CLOCKS + 1)-1:0] remaining = 0;

    assign active = remaining != 0;

    always @(posedge clock)
        if (signal)
            remaining <= HOLD_CLOCKS;
        else if (active)
            remaining <= remaining - 1;
endmodule
