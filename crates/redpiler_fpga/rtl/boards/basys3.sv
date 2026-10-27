// The top of the Basys 3 build: the oscillator, the PLL, the serial line, the controller, the
// circuit and the display. The display shows the low 16 bits of the tick count at the last request,
// or the fault code of the last failed request with its decimal point lit.
module basys3 (
    input logic clock,
    input logic serial_receive,
    output logic serial_transmit,
    output logic [6:0] segment_n,
    output logic [3:0] anode_n,
    output logic decimal_point_n
);
    localparam int CLOCKS_PER_BIT = board_pkg::OSCILLATOR_HERTZ / board_pkg::BAUD;

    logic board_clock;
    // The toolchain reads how fast the circuit may run under the name of this net, so the buffer
    // that drives it is in the top.
    logic circuit_clock;
    logic unbuffered_circuit_clock;
    logic circuit_clock_enable;

    logic received_valid;
    logic [7:0] received;
    logic transmit_valid;
    logic [7:0] transmit;
    logic transmit_ready;

    logic pll_enable;
    logic pll_write;
    logic [6:0] pll_address;
    logic [15:0] pll_write_data;
    logic pll_ready;
    logic [15:0] pll_read_data;
    logic pll_locked;
    logic pll_reset;

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

    logic [15:0] ticks_run;
    pkg::status_t last_status;
    logic faulted;

    assign faulted = last_status != pkg::OK;
    wire unused = paused;

    BUFG board_buffer (.I(clock), .O(board_clock));
    pll_drp #(.OSCILLATOR_HERTZ(board_pkg::OSCILLATOR_HERTZ)) pll (
        .oscillator(clock), .board_clock(board_clock), .access(pll_enable), .write(pll_write),
        .address(pll_address), .write_data(pll_write_data), .ready(pll_ready),
        .read_data(pll_read_data), .reset(pll_reset), .locked(pll_locked),
        .unbuffered(unbuffered_circuit_clock)
    );
    BUFGCE circuit_buffer (
        .I(unbuffered_circuit_clock), .CE(circuit_clock_enable), .O(circuit_clock)
    );

    uart_rx #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) receiver (
        .clock(board_clock), .line(serial_receive), .valid(received_valid), .data(received)
    );
    uart_tx #(.CLOCKS_PER_BIT(CLOCKS_PER_BIT)) transmitter (
        .clock(board_clock), .valid(transmit_valid), .data(transmit), .ready(transmit_ready),
        .line(serial_transmit)
    );
    controller controller (
        .board_clock(board_clock), .received_valid(received_valid), .received(received),
        .transmit_valid(transmit_valid), .transmit(transmit), .transmit_ready(transmit_ready),
        .pll_enable(pll_enable), .pll_write(pll_write), .pll_address(pll_address),
        .pll_write_data(pll_write_data), .pll_ready(pll_ready), .pll_read_data(pll_read_data),
        .pll_locked(pll_locked), .pll_reset(pll_reset),
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
    seven_segment display (
        .clock(board_clock), .digits(faulted ? 16'(last_status) : ticks_run), .decimal_point(faulted),
        .segment_n(segment_n), .anode_n(anode_n), .decimal_point_n(decimal_point_n)
    );
endmodule
