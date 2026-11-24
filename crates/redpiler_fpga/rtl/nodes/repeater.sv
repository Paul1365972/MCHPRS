// A repeater that turns on from an input that is off again by then pulses:
// it schedules its own tick to turn off.
module repeater #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic LOCKED = 0,
    parameter int PENDING_DELAY,
    parameter int FIRES
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
    logic unlocked_fire;
    logic pending;
    logic pulse;
    logic mismatch;
    logic schedule;

    assign firing = powered ? firing_on : firing_off;
    assign unlocked_fire = fire && !side_on;
    assign pulse = unlocked_fire && pkg::repeater_pulses(powered, default_on);
    assign mismatch = !side_on && powered != default_on;
    assign schedule = pulse || (!pending && mismatch);

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY), .FIRES(FIRES)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .mismatch(mismatch), .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) begin
        if (unlocked_fire) powered <= pkg::repeater_ticked(powered, default_on);
        locked <= side_on;
    end
endmodule
