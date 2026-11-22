// CircuitAnalysis::is_pulse_free selects this module.
// A pending tick implies that the input still differs from the output.
module repeater_pulse_free #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic LOCKED = 0,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing_off,
    input logic firing_on,
    input logic default_on,
    input logic side_on,
    output logic powered = POWERED,
    output logic locked = LOCKED,
    output logic [DELAY-1:0] countdown
);
    logic firing;
    logic fire;
    logic pending;
    logic schedule;

    assign firing = powered ? firing_on : firing_off;
    assign schedule = !side_on && !pending && powered != default_on;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) begin
        if (fire && !side_on) powered <= !powered;
        locked <= side_on;
    end
endmodule
