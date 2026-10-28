// It schedules only in the clock in which its input turns off,
// the only time `phase` updates a lit lamp: one whose input stays off already has its tick.
// `default_on_previous` starts low for that.
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
    logic fire;
    logic schedule;
    logic goes_dark;

    assign countdown_after_tick_start = tick_start ? countdown >> 1 : countdown;
    assign pending_later = |countdown_after_tick_start;
    assign turned_off = default_on_previous && !default_on;
    assign schedule = turned_off && !pending_later;
    assign goes_dark = fire && !default_on_previous;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending()
    );

    always_ff @(posedge clock) begin
        powered <= default_on || (powered && !goes_dark);
        default_on_previous <= default_on;
    end
endmodule
