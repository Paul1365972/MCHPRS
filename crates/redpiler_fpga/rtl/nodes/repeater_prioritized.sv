// The first priority fires on `tick_start`, before `timer` can register a due tick.
// Only the tick that ends a pulse can fire later.
module repeater_prioritized #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic LOCKED = 0,
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
    output logic pulsing = PULSING
);
    logic pending;
    logic schedule;
    logic [DELAY-1:0] next_countdown;
    logic fire;
    logic unlocked_fire;
    logic pulse;

    assign pending = |countdown || pulsing;
    assign schedule = !side_on && !pending && powered != default_on;

    always_comb begin
        next_countdown = countdown;
        if (schedule) next_countdown[DELAY-1] = 1'b1;
        if (pulsing) fire = firing_pulse && countdown == 0;
        else fire = tick_start && next_countdown[0];
    end

    assign unlocked_fire = fire && !side_on;
    assign pulse = unlocked_fire && pkg::repeater_pulses(powered, default_on);

    always_ff @(posedge clock) begin
        if (unlocked_fire) powered <= pkg::repeater_ticked(powered, default_on);
        locked <= side_on;
        countdown <= tick_start ? (next_countdown >> 1) : next_countdown;
        if (pulse) countdown[DELAY-1] <= 1'b1;
        if (fire) pulsing <= pulse;
    end
endmodule
