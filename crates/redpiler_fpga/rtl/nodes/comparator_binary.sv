// CircuitAnalysis::binary_strengths guarantees that the output has only two possible strengths.
module comparator_binary #(
    parameter int DELAY,
    parameter logic SUBTRACT,
    parameter logic FAR_VALID,
    parameter logic [3:0] FAR,
    parameter logic [3:0] HIGH,
    parameter logic [3:0] STRENGTH,
    parameter int PENDING_DELAY,
    parameter int FIRES
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic [3:0] default_strength,
    input logic [3:0] side_strength,
    output logic high = STRENGTH == HIGH,
    output logic [DELAY-1:0] countdown
);
    logic desired_high;
    logic mismatch;
    logic fire;
    logic pending;
    logic schedule;

    assign desired_high = HIGH == pkg::comparator_output(
        SUBTRACT, pkg::comparator_rear(FAR_VALID, FAR, default_strength), side_strength
    );
    assign mismatch = desired_high != high;
    assign schedule = !pending && mismatch;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY), .FIRES(FIRES)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) if (fire) high <= desired_high;
endmodule
