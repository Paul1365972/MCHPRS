// The pending ticks of a node.
//
// Bit k of `countdown` is a tick that falls due at the (k+1)-th tick start from now, so each tick
// start shifts it right by one. A tick scheduled in the clock of `tick_start` lands in the top bit
// after the shift, so it runs DELAY ticks later, like a tick scheduled later in the same tick.
// `due` holds a tick from the start at which it fell due until it fires, so a snapshot only needs
// to read `countdown`.
module timer #(
    parameter int DELAY,
    // The delay of the tick that is pending at compile time, or 0 for none.
    parameter int PENDING_DELAY,
    localparam logic [DELAY:0] PENDING_TICK = (DELAY + 1)'(1) << PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic schedule,
    input logic fire,
    output logic [DELAY-1:0] countdown = PENDING_TICK[DELAY:1],
    output logic due = 0
);
    always_ff @(posedge clock) begin
        if (tick_start) countdown <= countdown >> 1;
        if (schedule) countdown[DELAY-1] <= 1'b1;

        if (tick_start) due <= countdown[0];
        else if (fire) due <= 1'b0;
    end
endmodule
