module node_torch #(
    parameter POWERED = 0,
    parameter PREVIOUS_INPUT = 0,
    parameter PENDING = 0,
    parameter STALE = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire input_power,
    input wire changed,
    output reg powered = POWERED,
    output wire countdown,
    output wire [3:0] signal,
    output wire [3:0] next_signal
);
    reg previous_input = PREVIOUS_INPUT;
    reg due = 1'b0;
    wire update;
    wire fire = due && firing[3];
    wire pending = countdown || due && !fire;
    wire next_powered = fire ? !previous_input : powered;
    assign signal = {4{powered}};
    assign next_signal = {4{next_powered}};

    always @(posedge clock) begin
        previous_input <= input_power;
        powered <= next_powered;
        if (fire) due <= 1'b0;
        else if (begin_tick) due <= countdown;
    end

    node_settling #(.STALE(STALE)) settling (.clock(clock), .changed(changed), .update(update));
    node_countdown #(.DELAY(1), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick),
        .schedule(update && !pending && next_powered == input_power), .countdown(countdown)
    );
endmodule
