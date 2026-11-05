module node_button #(
    parameter POWERED = 0,
    parameter INDEX = 0,
    parameter INDEX_WIDTH = 1,
    parameter PENDING = 0
) (
    input wire clock,
    input wire begin_tick,
    input wire [3:0] firing,
    input wire input_nodes,
    input wire [31:0] input_index,
    input wire input_value,
    output reg powered = POWERED,
    output wire [9:0] countdown,
    output wire [3:0] next_signal
);
    wire due;
    wire fire = due && firing[3];
    wire selected = input_nodes && input_index[INDEX_WIDTH-1:0] == INDEX;
    wire pressed = selected && input_value && !powered;
    wire next_powered = pressed || powered && !fire;
    assign next_signal = {4{next_powered}};

    always @(posedge clock)
        powered <= next_powered;

    node_timer #(.DELAY(10), .PENDING(PENDING)) tick (
        .clock(clock), .begin_tick(begin_tick), .fire(fire), .schedule(pressed),
        .countdown(countdown), .due(due)
    );
endmodule
