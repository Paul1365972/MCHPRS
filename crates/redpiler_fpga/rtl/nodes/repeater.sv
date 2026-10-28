// A redstone repeater: it follows its input DELAY ticks later unless a side input locks it.
//
// The priority of its tick follows its output, so it stores none: a tick fires in the clock of
// `firing_off` if the repeater is off by then, and in the clock of `firing_on` if it is on.
// A repeater that turns on from an input that is off again by then pulses: it schedules its own
// tick to turn off.
module repeater #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic LOCKED,
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
    output logic [DELAY-1:0] countdown,
    output logic powered_next
);
    logic default_on_previous = 0;
    logic due;
    logic fire;
    logic unlocked_fire;
    logic pending;
    logic pulse;
    logic schedule;

    assign fire = due && (powered ? firing_on : firing_off);
    assign unlocked_fire = fire && !locked;
    assign pending = |countdown || (due && !fire);
    assign powered_next =
        unlocked_fire ? pkg::repeater_ticked(powered, default_on_previous) : powered;
    assign pulse = unlocked_fire && pkg::repeater_pulses(powered, default_on_previous);
    assign schedule = pulse || (!side_on && !pending && powered_next != default_on);

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .fire(fire),
        .countdown(countdown), .due(due)
    );

    always_ff @(posedge clock) begin
        powered <= powered_next;
        locked <= side_on;
        default_on_previous <= default_on;
    end
endmodule
