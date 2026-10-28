// A redstone comparator: one tick after its inputs change, it outputs the strength they give.
module comparator #(
    parameter int DELAY,
    parameter logic SUBTRACT,
    // A far input, such as a container read through a block, replaces a rear input below 15.
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
    logic due;
    logic fire;
    logic pending;
    logic schedule;

    assign rear_strength = pkg::comparator_rear(FAR_VALID, FAR, default_strength);
    assign desired = pkg::comparator_output(SUBTRACT, rear_strength, side_strength);
    assign fire = due && firing;
    assign pending = |countdown || (due && !fire);
    assign strength_next = fire ? desired_previous : strength;
    assign schedule = !pending && desired != strength_next;

    timer #(.DELAY(DELAY), .PENDING_DELAY(PENDING_DELAY)) timer (
        .clock(clock), .tick_start(tick_start), .schedule(schedule), .fire(fire),
        .countdown(countdown), .due(due)
    );

    always_ff @(posedge clock) begin
        strength <= strength_next;
        desired_previous <= desired;
    end
endmodule
