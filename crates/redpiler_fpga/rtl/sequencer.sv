// A tick runs in one clock, as one step per priority. Each step reads what the step before left,
// so what the last one changes reaches the other nodes only in the next clock: `settling` holds
// `idle` low for the clock after a tick and after a drive applies.
// A drive from the host only loads registers, so no path leads from the handshake into a node.
module sequencer (
    input logic clock,
    input logic hold,
    input logic run_start,
    input pkg::mode_t run_mode,
    input logic [pkg::COUNT_BITS-1:0] run_count,
    input logic drive_valid,
    input logic [pkg::NODE_BITS-1:0] drive_node,
    input pkg::drive_input_t drive_input,
    output logic idle,
    output logic [63:0] ticks = 0,
    output logic paused,
    output logic ticking = 0,
    output logic taken_valid = 0,
    output logic [pkg::NODE_BITS-1:0] taken_node = 0,
    output pkg::drive_input_t taken_input = 0
);
    pkg::mode_t mode = pkg::PAUSED;
    logic [pkg::COUNT_BITS-1:0] remaining = 0;
    logic [pkg::COUNT_BITS-1:0] period = 0;
    // Readiness is registered so the run counter stays off the tick path.
    logic ready = 0;
    logic beginning;
    logic applying = 0;
    logic settling = 0;

    assign idle = !ticking && !taken_valid && !applying && !settling;
    assign paused = mode == pkg::PAUSED;
    assign beginning = !hold && ready;

    always_ff @(posedge clock) begin
        ticking <= beginning;

        if (ticking) ticks <= ticks + 1'b1;

        unique case (mode)
            pkg::PAUSED: ready <= 0;
            pkg::UNLIMITED: ready <= 1;
            pkg::COUNTED: begin
                ready <= beginning ? remaining > 1 : remaining != 0;
                if (beginning) remaining <= remaining - 1'b1;
                if (remaining == 0) mode <= pkg::PAUSED;
            end
            pkg::PERIODIC: begin
                if (remaining == 1) begin
                    ready <= 1;
                    remaining <= period;
                end else begin
                    if (beginning) ready <= 0;
                    remaining <= remaining - 1'b1;
                end
            end
        endcase
        if (run_start) begin
            mode <= run_mode;
            remaining <= run_count;
            period <= run_count;
            ready <= 0;
        end

        taken_valid <= drive_valid;
        if (drive_valid) begin
            taken_node <= drive_node;
            taken_input <= drive_input;
        end
        applying <= taken_valid;
        settling <= ticking || applying;
    end
endmodule
