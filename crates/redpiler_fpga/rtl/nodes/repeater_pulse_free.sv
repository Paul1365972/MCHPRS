// A repeater that can never pulse, so it has no pulse path.
//
// Relies on `CircuitAnalysis::is_pulse_free`: while a tick is pending, the input differs from the
// output, so the tick toggles an unlocked repeater without reading the input.
module repeater_pulse_free #(
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
    logic due;
    logic fire;
    logic pending;
    logic schedule;

    assign fire = due && (powered ? firing_on : firing_off);
    assign pending = |countdown || (due && !fire);
    assign powered_next = powered ^ (fire && !locked);
    assign schedule = !side_on && !pending && powered_next != default_on;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .fire(fire),
        .countdown(countdown), .due(due)
    );

    always_ff @(posedge clock) begin
        powered <= powered_next;
        locked <= side_on;
    end
endmodule
