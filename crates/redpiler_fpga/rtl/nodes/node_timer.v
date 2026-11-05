// A scheduled tick counts down to the tick start that makes it due; it stays due until the node
// fires in the phase of its priority.
module node_timer #(
    parameter DELAY = 1,
    parameter PENDING = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire schedule,
    input wire fire,
    output wire [DELAY-1:0] countdown,
    output reg due = 1'b0,
    output wire pending
);
    assign pending = |countdown || due && !fire;

    always @(posedge clock)
        if (fire) due <= 1'b0;
        else if (begin_tick) due <= countdown[0];

    node_countdown #(.DELAY(DELAY), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick), .schedule(schedule), .countdown(countdown)
    );
endmodule
