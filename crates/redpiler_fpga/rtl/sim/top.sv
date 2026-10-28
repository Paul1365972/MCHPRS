// The top of the Verilator build: the controller and the circuit. `sim/main.cpp` drives its
// clocks and serial bytes.
module top (
    input logic board_clock,
    input logic circuit_clock,
    output logic circuit_paused,
    input logic received_valid,
    input logic [7:0] received,
    output logic transmit_valid,
    output logic [7:0] transmit,
    input logic transmit_ready
);
    logic hold;
    logic run_start;
    pkg::mode_t run_mode;
    logic [pkg::COUNT_BITS-1:0] run_count;
    logic idle;
    logic [63:0] tick_count;
    logic paused;
    logic drive_valid;
    logic [23:0] drive_node;
    logic [5:0] drive_input;
    logic shown_read;
    logic [circuit_pkg::STATE_BITS-1:0] state;

    assign circuit_paused = paused;

    controller controller (
        .board_clock(board_clock), .received_valid(received_valid), .received(received),
        .transmit_valid(transmit_valid), .transmit(transmit), .transmit_ready(transmit_ready),
        .circuit_clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count),
        .drive_valid(drive_valid), .drive_node(drive_node), .drive_input(drive_input),
        .shown_read(shown_read), .state(state)
    );
    circuit circuit (
        .clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count), .paused(paused),
        .drive_valid(drive_valid), .drive_node(drive_node), .drive_input(drive_input),
        .shown_read(shown_read), .state(state)
    );
endmodule
