// A redstone lamp: it lights at once and goes dark one tick after its input turns off.
//
// A lit lamp whose input is off would schedule a tick even though one is already due in this
// redstone tick. So it schedules only in the clock in which its input turns off, the only time
// `phase` updates it then. `default_on_previous` starts low for that.
module lamp #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic default_on,
    output logic powered = POWERED,
    output logic [DELAY-1:0] countdown
);
    logic default_on_previous = 0;
    logic [DELAY-1:0] countdown_after_tick_start;
    logic pending_later;
    logic turned_off;
    logic due;
    logic fire;
    logic schedule;
    logic goes_dark;

    assign countdown_after_tick_start = tick_start ? countdown >> 1 : countdown;
    assign pending_later = |countdown_after_tick_start;
    assign turned_off = default_on_previous && !default_on;
    assign fire = due && firing;
    assign schedule = turned_off && !pending_later;
    assign goes_dark = fire && !default_on_previous;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .fire(fire),
        .countdown(countdown), .due(due)
    );

    always_ff @(posedge clock) begin
        powered <= default_on || (powered && !goes_dark);
        default_on_previous <= default_on;
    end
endmodule
