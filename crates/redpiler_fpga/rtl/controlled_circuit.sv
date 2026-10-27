module controlled_circuit (
    input logic board_clock,
    input logic received_valid,
    input logic [7:0] received,
    output logic transmit_valid,
    output logic [7:0] transmit,
    input logic transmit_ready,

    output logic pll_enable,
    output logic pll_write,
    output logic [6:0] pll_address,
    output logic [15:0] pll_write_data,
    input logic pll_ready,
    input logic [15:0] pll_read_data,
    input logic pll_locked,
    output logic pll_reset,
    output logic circuit_clock_enable,

    input logic circuit_clock,
    output logic paused,

    output logic [15:0] ticks_run,
    output pkg::status_t last_status
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
        .pll_enable(pll_enable), .pll_write(pll_write), .pll_address(pll_address),
        .pll_write_data(pll_write_data), .pll_ready(pll_ready), .pll_read_data(pll_read_data),
        .pll_locked(pll_locked), .pll_reset(pll_reset), .circuit_clock_enable(circuit_clock_enable),
        .circuit_clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count), .drive_valid(drive_valid),
        .drive_node(drive_node), .drive_input(drive_input), .shown_read(shown_read), .state(state),
        .ticks_run(ticks_run), .last_status(last_status)
    );
    circuit circuit (
        .clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count), .paused(paused),
        .drive_valid(drive_valid), .drive_node(drive_node), .drive_input(drive_input),
        .shown_read(shown_read), .state(state)
    );
endmodule
