module node_torch #(
    parameter POWERED = 0,
    parameter PREVIOUS_INPUT = 0,
    parameter PENDING = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire input_power,
    output reg powered = POWERED,
    output wire countdown,
    output wire [3:0] next_signal
);
    reg previous_input = PREVIOUS_INPUT;
    wire due, pending;
    wire fire = due && firing[3];
    wire next_powered = fire ? !previous_input : powered;
    assign next_signal = {4{next_powered}};

    always @(posedge clock) begin
        previous_input <= input_power;
        powered <= next_powered;
    end

    node_timer #(.DELAY(1), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick), .fire(fire),
        .schedule(!pending && next_powered == input_power),
        .countdown(countdown), .due(due), .pending(pending)
    );
endmodule
