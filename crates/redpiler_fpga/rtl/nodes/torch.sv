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
    output logic [DELAY-1:0] countdown
);
    logic mismatch;
    logic fire;
    logic pending;
    logic schedule;

    assign mismatch = powered == default_on;
    assign schedule = !pending && mismatch;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) if (fire) powered <= !default_on;
endmodule
