// A pressed button always has a pending tick, so a press need not ask whether it is pressed.
module button #(
    parameter logic [pkg::NODE_BITS-1:0] INDEX,
    parameter int DELAY,
    parameter logic POWERED,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic taken_valid,
    input logic [pkg::NODE_BITS-1:0] taken_node,
    output logic powered = POWERED,
    output logic [DELAY-1:0] countdown
);
    logic pressed = 0;
    logic fire;
    logic pending;
    logic schedule;

    assign schedule = pressed && !pending;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .mismatch(1'b0), .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) begin
        pressed <= taken_valid && taken_node == INDEX;
        powered <= pressed || (powered && !fire);
    end
endmodule
