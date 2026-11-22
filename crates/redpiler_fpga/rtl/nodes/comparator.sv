module comparator #(
    parameter int DELAY,
    parameter logic SUBTRACT,
    parameter logic FAR_VALID,
    parameter logic [3:0] FAR,
    parameter logic [3:0] STRENGTH,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic [3:0] default_strength,
    input logic [3:0] side_strength,
    output logic [3:0] strength = STRENGTH,
    output logic [DELAY-1:0] countdown
);
    logic [3:0] desired;
    logic mismatch;
    logic fire;
    logic pending;
    logic schedule;

    assign desired = pkg::comparator_output(
        SUBTRACT, pkg::comparator_rear(FAR_VALID, FAR, default_strength), side_strength
    );
    assign mismatch = desired != strength;
    assign schedule = !pending && mismatch;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) if (fire) strength <= desired;
endmodule
