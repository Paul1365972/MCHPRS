// Bit k of the countdown means the tick becomes due at the (k+1)-th tick start from now.
module node_countdown #(
    parameter DELAY = 1,
    parameter PENDING = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire schedule,
    output reg [DELAY-1:0] countdown = (1 << PENDING) >> 1
);
    localparam [DELAY-1:0] SCHEDULED = 1 << (DELAY - 1);

    always @(posedge clock)
        countdown <= (begin_tick ? countdown >> 1 : countdown) | (schedule ? SCHEDULED : 0);
endmodule
