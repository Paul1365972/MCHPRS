// Countdown bit k falls due on the (k+1)th tick boundary.
// Scheduling on that boundary still lands in the top bit after the shift.
// Snapshots read the countdown between ticks.
module timer #(
    parameter int DELAY,
    parameter int PENDING_DELAY,
    localparam logic [DELAY:0] PENDING_TICK = (DELAY + 1)'(1) << PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic schedule,
    // High in the clock of the node's priority; `fire` is high when the node itself fires.
    input logic firing,
    output logic [DELAY-1:0] countdown = PENDING_TICK[DELAY:1],
    output logic fire,
    output logic pending
);
    logic due = 0;

    assign fire = due && firing;
    assign pending = |countdown || (due && !fire);

    always_ff @(posedge clock) begin
        if (tick_start) countdown <= countdown >> 1;
        if (schedule) countdown[DELAY-1] <= 1'b1;

        if (tick_start) due <= countdown[0];
        else if (fire) due <= 1'b0;
    end
endmodule
