// The serial line, the display and the lights run on the board's 100 MHz oscillator, the circuit and
// its controller on the circuit clock, which the PLL derives from an oscillator locked to the
// board's. Placement and routing time the 100 MHz that the PLL's parameters give here; the toolchain
// then sets the PLL to the fastest clock the routed circuit meets. Every tenth of a second the
// display asks the circuit's side for its tick count and fault code. Without SEVEN_SEGMENT_DISPLAY
// the display, its views and the buttons that cycle them are left out to save logic.
module basys3 #(
    parameter SEVEN_SEGMENT_DISPLAY = 1
) (
    input wire clock,
    input wire serial_receive,
    output wire serial_transmit,
    input wire button_center,
    input wire button_left,
    input wire button_right,
    output wire [15:0] led,
    output wire [6:0] segment,
    output wire [3:0] anode,
    output wire decimal_point
);
    localparam BOARD_CLOCK_HERTZ = 100_000_000;
    localparam REFRESHES_PER_SECOND = 10;
    localparam BOARD_CLOCKS_PER_REFRESH = BOARD_CLOCK_HERTZ / REFRESHES_PER_SECOND;
    localparam ACTIVITY_CLOCKS = BOARD_CLOCK_HERTZ / 20;
    localparam [1:0] TICK_RATE_VIEW = 0, TICK_COUNT_VIEW = 1, FAULT_VIEW = 2;
    localparam [7:0] DASH = 8'h40, LETTER_E = 8'h79;

    wire board_clock, circuit_clock, feedback, divided, locked;
    wire ticking, halted, clear_fault;
    wire ticking_seen, receiving_seen, ticking_shown, receiving, transmitting;
    wire [3:0] fault_code;
    wire [63:0] tick_count;
    wire [2:0] view_lights;

    BUFG board_buffer (.I(clock), .O(board_clock));
    PLLE2_BASE #(.CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT(16), .CLKOUT0_DIVIDE(16)) pll (
        .CLKIN1(clock), .CLKFBIN(feedback), .CLKFBOUT(feedback), .CLKOUT0(divided),
        .RST(1'b0), .PWRDWN(1'b0), .LOCKED(locked)
    );
    BUFGCE circuit_buffer (.I(divided), .CE(locked), .O(circuit_clock));

    circuit_system system (
        .clock(circuit_clock), .serial_clock(board_clock), .serial_receive(serial_receive),
        .clear_fault(clear_fault), .serial_transmit(serial_transmit), .ticking(ticking),
        .halted(halted), .tick_count(tick_count), .fault_code(fault_code)
    );
    push_button center (.clock(circuit_clock), .button(button_center), .pressed(clear_fault));

    reg [26:0] heartbeat = 0;

    always @(posedge board_clock)
        heartbeat <= heartbeat + 1;

    assign led = {
        fault_code != 0, view_lights,
        7'b0, transmitting, receiving, halted, ticking_shown, heartbeat[26]
    };

    generate
        if (SEVEN_SEGMENT_DISPLAY) begin : views
            wire snapshot_due, snapshot_arrived, previous_view, next_view;
            wire [3:0] shown_fault_code;
            wire [63:0] counted_ticks;
            wire [79:0] digits;
            wire [31:0] number;
            reg [23:0] refresh_clocks = 0;
            reg [3:0] snapshots = 0;
            reg [1:0] view = TICK_RATE_VIEW;
            reg [31:0] second_start = 0;
            reg [31:0] tick_rate = 0;
            wire refresh_due = refresh_clocks == BOARD_CLOCKS_PER_REFRESH - 1;
            wire [63:0] shown_value = view == TICK_RATE_VIEW ? tick_rate
                : view == TICK_COUNT_VIEW ? counted_ticks
                : shown_fault_code;
            wire [31:0] glyphs = view != FAULT_VIEW ? number
                : shown_fault_code == 0 ? {4{DASH}}
                : {LETTER_E, number[23:0]};

            assign view_lights = {view == TICK_RATE_VIEW, view == TICK_COUNT_VIEW, view == FAULT_VIEW};

            always @(posedge board_clock) begin
                refresh_clocks <= refresh_due ? 0 : refresh_clocks + 1;
                if (snapshot_arrived) begin
                    snapshots <= snapshots == REFRESHES_PER_SECOND - 1 ? 0 : snapshots + 1;
                    if (snapshots == 0) begin
                        second_start <= counted_ticks[31:0];
                        tick_rate <= counted_ticks[31:0] - second_start;
                    end
                end
                if (previous_view)
                    view <= view == TICK_RATE_VIEW ? FAULT_VIEW : view - 1;
                else if (next_view)
                    view <= view == FAULT_VIEW ? TICK_RATE_VIEW : view + 1;
            end

            value_crossing #(.WIDTH(68)) snapshot (
                .source_clock(circuit_clock), .update(snapshot_due), .value({fault_code, tick_count}),
                .destination_clock(board_clock), .updated(snapshot_arrived),
                .crossed({shown_fault_code, counted_ticks})
            );
            pulse_crossing snapshot_request (
                .source_clock(board_clock), .pulse(refresh_due), .destination_clock(circuit_clock),
                .crossed(snapshot_due)
            );
            push_button left (.clock(board_clock), .button(button_left), .pressed(previous_view));
            push_button right (.clock(board_clock), .button(button_right), .pressed(next_view));
            decimal_digits converter (
                .clock(board_clock), .start(snapshot_arrived || previous_view || next_view),
                .value(shown_value), .digits(digits)
            );
            number_glyphs formatter (.digits(digits), .glyphs(number));
            seven_segment_display display (
                .clock(board_clock), .glyphs(glyphs), .segment(segment), .anode(anode),
                .decimal_point(decimal_point)
            );
        end else begin : dark
            assign view_lights = 0;
            assign segment = 7'h7f;
            assign anode = 4'hf;
            assign decimal_point = 1;
        end
    endgenerate

    synchronizer ticking_synchronizer (
        .clock(board_clock), .signal(ticking), .synchronized(ticking_seen)
    );
    synchronizer receive_synchronizer (
        .clock(board_clock), .signal(!serial_receive), .synchronized(receiving_seen)
    );
    activity #(.HOLD_CLOCKS(ACTIVITY_CLOCKS)) ticking_activity (
        .clock(board_clock), .signal(ticking_seen), .active(ticking_shown)
    );
    activity #(.HOLD_CLOCKS(ACTIVITY_CLOCKS)) receive_activity (
        .clock(board_clock), .signal(receiving_seen), .active(receiving)
    );
    activity #(.HOLD_CLOCKS(ACTIVITY_CLOCKS)) transmit_activity (
        .clock(board_clock), .signal(!serial_transmit), .active(transmitting)
    );
endmodule
