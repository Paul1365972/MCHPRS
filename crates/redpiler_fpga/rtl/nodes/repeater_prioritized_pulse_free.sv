// CircuitAnalysis::is_pulse_free selects this module.
// A pending tick implies that the input still differs from the output.
module repeater_prioritized_pulse_free #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic LOCKED = 0,
    parameter int PENDING_DELAY,
    localparam logic [DELAY:0] PENDING_TICK = (DELAY + 1)'(1) << PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic default_on,
    input logic side_on,
    output logic powered = POWERED,
    output logic locked = LOCKED,
    output logic [DELAY-1:0] countdown = PENDING_TICK[DELAY:1],
    output logic powered_next
);
    logic fire;
    logic pending;
    logic schedule;

    assign fire = tick_start && countdown[0];
    assign pending = |countdown && !fire;
    assign powered_next = powered ^ (fire && !locked);
    assign schedule = !side_on && !pending && powered_next != default_on;

    always_ff @(posedge clock) begin
        powered <= powered_next;
        locked <= side_on;
        if (tick_start) countdown <= countdown >> 1;
        if (schedule) countdown[DELAY-1] <= 1'b1;
    end
endmodule
