module node_lamp #(
    parameter POWERED = 0,
    parameter PREVIOUS_INPUT = 0,
    parameter PENDING = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire input_power,
    output reg powered = POWERED,
    output wire [1:0] countdown
);
    reg previous_input = PREVIOUS_INPUT;
    wire due, pending;
    wire fire = due && firing[3];
    wire stays_powered = powered && !(fire && !previous_input);

    always @(posedge clock) begin
        previous_input <= input_power;
        powered <= stays_powered || input_power;
    end

    node_timer #(.DELAY(2), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick), .fire(fire),
        .schedule(stays_powered && !input_power && !pending),
        .countdown(countdown), .due(due), .pending(pending)
    );
endmodule
