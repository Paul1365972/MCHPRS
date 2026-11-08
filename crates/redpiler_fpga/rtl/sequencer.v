// A tick takes three clocks: priority 0, then priorities 1 and 2 together, then 3. firing holds one
// bit per priority and shifts it each clock, beginning a tick at priority 0 and skipping bit 2, so
// nodes of priority 2 fire on bit 1. Each of the GROUPS groups of nodes reads its own copy of
// firing, which placement can put among them instead of routing one register across the device, so
// whether a tick begins is decided in the clock before. set_input may only rise while run is low,
// and tick_limit may only change between ticks.
module fpga_sequencer #(
    parameter GROUPS = 1
) (
    input wire clock,
    input wire set_input,
    input wire run,
    input wire [63:0] tick_limit,
    output wire input_nodes,
    output wire begin_tick,
    output wire [GROUPS * 4 - 1:0] firing,
    output wire busy,
    output reg [63:0] tick_count = 0
);
    reg [3:0] phase = 0;
    reg at_limit = 1;
    reg limit_after_tick = 0;
    wire idle = phase == 0;
    wire beginning = run && (phase[3] ? !limit_after_tick : idle && !at_limit);

    assign input_nodes = idle && set_input;
    assign begin_tick = phase[0];
    assign busy = !idle;

    always @(posedge clock) begin
        phase <= {phase[1], 1'b0, phase[0], beginning};
        tick_count <= tick_count + phase[3];
        at_limit <= phase[3] ? limit_after_tick : tick_count == tick_limit;
        limit_after_tick <= tick_count + 1 == tick_limit;
    end

    genvar group;
    generate
        for (group = 0; group < GROUPS; group = group + 1) begin : copies
            (* keep *) fpga_firing_copy copy (
                .clock(clock), .beginning(beginning), .firing(firing[group * 4 +: 4])
            );
        end
    endgenerate
endmodule
