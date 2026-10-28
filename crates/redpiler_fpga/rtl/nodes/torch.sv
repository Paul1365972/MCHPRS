module torch #(
    parameter int DELAY,
    parameter logic POWERED,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic default_on,
    output logic powered = POWERED,
    output logic [DELAY-1:0] countdown,
    output logic powered_next
);
    logic default_on_previous = 0;
    logic fire;
    logic pending;
    logic schedule;

    assign powered_next = fire ? !default_on_previous : powered;
    assign schedule = !pending && powered_next == default_on;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) begin
        powered <= powered_next;
        default_on_previous <= default_on;
    end
endmodule
