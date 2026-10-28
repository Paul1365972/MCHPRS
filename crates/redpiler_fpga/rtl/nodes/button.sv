// A button: a press powers it, and its tick releases it.
//
// A pressed button always has a pending tick, so a press need not ask whether it is pressed.
module button #(
    parameter logic [23:0] INDEX,
    parameter int DELAY,
    parameter logic POWERED,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic taken_valid,
    input logic [23:0] taken_node,
    output logic powered = POWERED,
    output logic [DELAY-1:0] countdown,
    output logic powered_next
);
    logic pressed = 0;
    logic due;
    logic fire;
    logic pending;
    logic schedule;

    assign fire = due && firing;
    assign pending = |countdown || (due && !fire);
    assign schedule = pressed && !pending;
    assign powered_next = pressed || (powered && !fire);

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .fire(fire),
        .countdown(countdown), .due(due)
    );

    always_ff @(posedge clock) begin
        pressed <= taken_valid && taken_node == INDEX;
        powered <= powered_next;
    end
endmodule
