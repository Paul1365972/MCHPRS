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
    reg due = 1'b0;
    wire fire = due && firing[3];
    wire pending = |countdown || due && !fire;
    wire stays_powered = powered && !(fire && !previous_input);

    always @(posedge clock) begin
        previous_input <= input_power;
        powered <= stays_powered || input_power;
        if (fire) due <= 1'b0;
        else if (begin_tick) due <= countdown[0];
    end

    node_countdown #(.DELAY(2), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick),
        .schedule(stays_powered && !input_power && !pending), .countdown(countdown)
    );
endmodule
