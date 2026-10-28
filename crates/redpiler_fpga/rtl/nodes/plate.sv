// A pressure plate: touching it powers it, and its tick rechecks whether it is still touched.
//
// A pressed plate always has a pending tick, so a touch need not ask whether it is pressed.
module plate #(
    parameter logic [23:0] INDEX,
    parameter int DELAY,
    parameter logic POWERED,
    parameter logic TOUCHED,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic taken_valid,
    input logic [23:0] taken_node,
    input pkg::drive_input_t taken_input,
    output logic powered = POWERED,
    output logic touched = TOUCHED,
    output logic [DELAY-1:0] countdown,
    output logic powered_next
);
    logic driven = 0;
    logic touched_next;
    logic pressed;
    logic released;
    logic rechecked_touched;
    logic due;
    logic fire;
    logic pending;
    logic schedule;

    always_comb begin
        touched_next = touched;
        if (driven)
            touched_next = taken_input.kind == pkg::INPUT_TOGGLE ? !touched : taken_input.value[0];
    end

    assign fire = due && firing;
    assign pending = |countdown || (due && !fire);
    assign pressed = driven && touched_next;
    assign released = fire && !touched;
    assign rechecked_touched = fire && powered && touched;
    assign schedule = (pressed && !pending) || rechecked_touched;
    assign powered_next = pressed || (powered && !released);

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .fire(fire),
        .countdown(countdown), .due(due)
    );

    always_ff @(posedge clock) begin
        driven <= taken_valid && taken_node == INDEX;
        powered <= powered_next;
        touched <= touched_next;
    end
endmodule
