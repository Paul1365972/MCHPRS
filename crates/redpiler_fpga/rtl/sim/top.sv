// The top of the Verilator build: the controller and the circuit, with a model of the PLL's
// registers in place of the real PLL. `sim/main.cpp` drives its clocks and serial bytes.
module top (
    input logic board_clock,
    input logic circuit_clock,
    output logic circuit_clock_enable,
    output logic circuit_paused,
    input logic received_valid,
    input logic [7:0] received,
    output logic transmit_valid,
    output logic [7:0] transmit,
    input logic transmit_ready,
    output logic [15:0] ticks_run,
    output pkg::status_t last_status
);
    localparam int LOCK_CLOCKS = 20;

    logic pll_enable;
    logic pll_write;
    logic [6:0] pll_address;
    logic [15:0] pll_write_data;
    logic pll_ready = 0;
    logic [15:0] pll_read_data = 0;
    logic pll_reset;
    logic [15:0] pll_registers[128];
    logic [4:0] lock_clocks_left = 5'(LOCK_CLOCKS);

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
        .pll_enable(pll_enable), .pll_write(pll_write), .pll_address(pll_address),
        .pll_write_data(pll_write_data), .pll_ready(pll_ready), .pll_read_data(pll_read_data),
        .pll_locked(lock_clocks_left == 0), .pll_reset(pll_reset),
        .circuit_clock_enable(circuit_clock_enable),
        .circuit_clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count),
        .drive_valid(drive_valid), .drive_node(drive_node), .drive_input(drive_input),
        .shown_read(shown_read), .state(state), .ticks_run(ticks_run), .last_status(last_status)
    );
    circuit circuit (
        .clock(circuit_clock), .hold(hold), .run_start(run_start), .run_mode(run_mode),
        .run_count(run_count), .idle(idle), .tick_count(tick_count), .paused(paused),
        .drive_valid(drive_valid), .drive_node(drive_node), .drive_input(drive_input),
        .shown_read(shown_read), .state(state)
    );

    always_ff @(posedge board_clock) begin
        pll_ready <= pll_enable;
        if (pll_enable) begin
            pll_read_data <= pll_registers[pll_address];
            if (pll_write) pll_registers[pll_address] <= pll_write_data;
        end
        if (pll_reset) lock_clocks_left <= 5'(LOCK_CLOCKS);
        else if (lock_clocks_left != 0) lock_clocks_left <= lock_clocks_left - 1'b1;
    end
endmodule
