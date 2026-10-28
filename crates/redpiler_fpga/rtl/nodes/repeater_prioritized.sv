// A repeater that faces into a diode, which gives its ticks the first priority.
//
// Its ticks fire in the clock of `tick_start` itself, so it needs no `due` register and keeps its
// own countdown instead of a `timer`. Only the tick that ends a pulse has another priority:
// `pulsing` marks it, and it fires in the clock of `firing_pulse` once the countdown is empty.
module repeater_prioritized #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic LOCKED,
    parameter int PENDING_DELAY,
    parameter logic PULSING,
    localparam logic [DELAY:0] PENDING_TICK = (DELAY + 1)'(1) << PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing_pulse,
    input logic default_on,
    input logic side_on,
    output logic powered = POWERED,
    output logic locked = LOCKED,
    output logic [DELAY-1:0] countdown = PENDING_TICK[DELAY:1],
    output logic pulsing = PULSING,
    output logic powered_next
);
    logic default_on_previous = 0;
    logic fire;
    logic unlocked_fire;
    logic pending;
    logic pulse;
    logic schedule;

    always_comb begin
        if (pulsing) fire = firing_pulse && countdown == 0;
        else fire = tick_start && countdown[0];
    end

    assign unlocked_fire = fire && !locked;
    assign pending = |countdown && !fire;
    assign powered_next =
        unlocked_fire ? pkg::repeater_ticked(powered, default_on_previous) : powered;
    assign pulse = unlocked_fire && pkg::repeater_pulses(powered, default_on_previous);
    assign schedule = pulse || (!side_on && !pending && powered_next != default_on);

    always_ff @(posedge clock) begin
        powered <= powered_next;
        locked <= side_on;
        default_on_previous <= default_on;
        if (tick_start) countdown <= countdown >> 1;
        if (schedule) countdown[DELAY-1] <= 1'b1;
        if (fire) pulsing <= pulse;
    end
endmodule
