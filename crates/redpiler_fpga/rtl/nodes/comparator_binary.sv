// A comparator that only ever outputs one of two strengths, so it stores one bit: whether it
// outputs HIGH. `CircuitAnalysis::binary_strengths` finds these comparators.
module comparator_binary #(
    parameter int DELAY,
    parameter logic SUBTRACT,
    parameter logic FAR_VALID,
    parameter logic [3:0] FAR,
    parameter logic [3:0] HIGH,
    parameter logic [3:0] STRENGTH,
    parameter int PENDING_DELAY
) (
    input logic clock,
    input logic tick_start,
    input logic firing,
    input logic [3:0] default_strength,
    input logic [3:0] side_strength,
    output logic high = STRENGTH == HIGH,
    output logic [DELAY-1:0] countdown,
    output logic high_next
);
    logic [3:0] rear_strength;
    logic desired_high;
    logic desired_high_previous = 0;
    logic due;
    logic fire;
    logic pending;
    logic schedule;

    assign rear_strength = pkg::comparator_rear(FAR_VALID, FAR, default_strength);
    assign desired_high = HIGH == pkg::comparator_output(SUBTRACT, rear_strength, side_strength);
    assign fire = due && firing;
    assign pending = |countdown || (due && !fire);
    assign high_next = fire ? desired_high_previous : high;
    assign schedule = !pending && desired_high != high_next;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .fire(fire),
        .countdown(countdown), .due(due)
    );

    always_ff @(posedge clock) begin
        high <= high_next;
        desired_high_previous <= desired_high;
    end
endmodule
