module fpga_sequencer(
    input wire clock,
    input wire set_input,
    input wire run,
    input wire [63:0] tick_limit,
    output wire input_nodes,
    output wire begin_tick,
    output wire [3:0] firing,
    output wire busy,
    output reg [63:0] tick_count = 0
);
    reg running = 0;
    reg [1:0] phase = 0;
    reg at_limit = 1;
    wire start = !running && !set_input && run && !at_limit;
    wire active = start || running;
    wire completing = active && phase == 3;
    wire [63:0] next_tick_count = tick_count + completing;

    assign busy = running;
    assign input_nodes = !running && set_input;
    assign begin_tick = start;
    assign firing = active ? 4'b0001 << phase : 4'b0000;

    always @(posedge clock) begin
        tick_count <= next_tick_count;
        at_limit <= next_tick_count == tick_limit;
        if (active) begin
            running <= !completing;
            phase <= phase + 1'b1;
        end
    end
endmodule
