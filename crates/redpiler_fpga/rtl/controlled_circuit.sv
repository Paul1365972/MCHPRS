module controlled_circuit (
    input logic board_clock,
    input logic received_valid,
    input logic [7:0] received,
    output logic transmit_valid,
    output logic [7:0] transmit,
    input logic transmit_ready,

    input logic circuit_clock,
    output logic paused
);
    logic hold;
    logic run_start;
    pkg::mode_t run_mode;
    logic [pkg::COUNT_BITS-1:0] run_count;
    logic idle;
    logic [63:0] tick_count;
    logic drive_valid;
    logic [pkg::NODE_BITS-1:0] drive_node;
    pkg::drive_input_t drive_input;
    logic shown_read;
    logic [circuit_pkg::STATE_BITS-1:0] state;

    controller controller (
        .board_clock(board_clock), .received_valid(received_valid), .received(received),
        .transmit_valid(transmit_valid), .transmit(transmit), .transmit_ready(transmit_ready),
        .circuit_clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count), .drive_valid(drive_valid),
        .drive_node(drive_node), .drive_input(drive_input), .shown_read(shown_read), .state(state)
    );
    circuit circuit (
        .clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count), .paused(paused),
        .drive_valid(drive_valid), .drive_node(drive_node), .drive_input(drive_input),
        .shown_read(shown_read), .state(state)
    );
endmodule
