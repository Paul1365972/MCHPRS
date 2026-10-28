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
    output logic [DELAY-1:0] countdown,
    output logic [3:0] strength_next
);
    logic [3:0] rear_strength;
    logic [3:0] desired;
    logic [3:0] desired_previous = 0;
    logic fire;
    logic pending;
    logic schedule;

    assign rear_strength = pkg::comparator_rear(FAR_VALID, FAR, default_strength);
    assign desired = pkg::comparator_output(SUBTRACT, rear_strength, side_strength);
    assign strength_next = fire ? desired_previous : strength;
    assign schedule = !pending && desired != strength_next;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .firing(firing),
        .countdown(countdown), .fire(fire), .pending(pending)
    );

    always_ff @(posedge clock) begin
        strength <= strength_next;
        desired_previous <= desired;
    end
endmodule
