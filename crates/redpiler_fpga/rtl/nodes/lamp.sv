// It schedules only in the clock in which `default_on` turns off,
// the only time `phase` updates a lit lamp: one whose input stays off already has its tick.
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
    logic was_on = 0;
    logic pending_later;
    logic turned_off;
    logic fire;
    logic schedule;
    logic goes_dark;

    assign pending_later = |countdown;
    assign turned_off = was_on && !default_on;
    assign schedule = turned_off && !pending_later;
    assign goes_dark = fire && !default_on;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending()
    );

    always_ff @(posedge clock) begin
        powered <= default_on || (powered && !goes_dark);
        was_on <= default_on;
    end
endmodule
