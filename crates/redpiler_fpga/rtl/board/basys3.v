// The serial line, the display and the lights run on the board's 100 MHz oscillator, the rest on the
// circuit clock, which the PLL divides from a 1.6 GHz oscillator locked to the board's. Placement and
// routing aim at the 100 MHz that CLKOUT0_DIVIDE gives here; the toolchain then replaces the divider
// with the smallest one whose clock the routed circuit meets.
module basys3 (
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
    wire ticking, halted, clear_fault, previous_view, next_view, refresh, second_started;
    wire [3:0] fault_code;
    wire [63:0] tick_count;
    wire [79:0] digits;
    wire [31:0] number;
    BUFG board_buffer (.I(clock), .O(board_clock));
    PLLE2_BASE #(.CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT(16), .CLKOUT0_DIVIDE(16)) pll (
        .CLKIN1(clock), .CLKFBIN(feedback), .CLKFBOUT(feedback), .CLKOUT0(divided),
        .RST(1'b0), .PWRDWN(1'b0), .LOCKED(locked)
    );
    BUFGCE circuit_buffer (.I(divided), .CE(locked), .O(circuit_clock));

    reg [26:0] heartbeat = 0;
    reg [23:0] refresh_clocks = 0;
    reg [3:0] refreshes = 0;
    wire refresh_due = refresh_clocks == BOARD_CLOCKS_PER_REFRESH - 1;
    wire ticking_seen, receiving_seen, ticking_shown, receiving, transmitting;
    wire [31:0] shown_glyphs;

    always @(posedge board_clock) begin
        heartbeat <= heartbeat + 1;
        refresh_clocks <= refresh_due ? 0 : refresh_clocks + 1;
        if (refresh_due)
            refreshes <= refreshes == REFRESHES_PER_SECOND - 1 ? 0 : refreshes + 1;
    end

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
    seven_segment_display display (
        .clock(board_clock), .glyphs(shown_glyphs), .segment(segment), .anode(anode),
        .decimal_point(decimal_point)
    );

    reg [1:0] view = TICK_RATE_VIEW;
    reg [63:0] second_start = 0;
    reg [63:0] tick_rate = 0;
    wire second = refresh && second_started;
    wire [63:0] shown_value = view == TICK_RATE_VIEW ? tick_rate
        : view == TICK_COUNT_VIEW ? tick_count
        : fault_code;
    wire [31:0] glyphs = view != FAULT_VIEW ? number
        : fault_code == 0 ? {4{DASH}}
        : {LETTER_E, number[23:0]};

    always @(posedge circuit_clock) begin
        if (second) begin
            second_start <= tick_count;
            tick_rate <= tick_count - second_start;
        end
        if (previous_view)
            view <= view == TICK_RATE_VIEW ? FAULT_VIEW : view - 1;
        else if (next_view)
            view <= view == FAULT_VIEW ? TICK_RATE_VIEW : view + 1;
    end

    assign led = {
        fault_code != 0, view == TICK_RATE_VIEW, view == TICK_COUNT_VIEW, view == FAULT_VIEW,
        7'b0, transmitting, receiving, halted, ticking_shown, heartbeat[26]
    };

    value_crossing time_base (
        .source_clock(board_clock), .update(refresh_due), .value(refreshes == 0),
        .destination_clock(circuit_clock), .updated(refresh), .crossed(second_started)
    );
    value_crossing #(.WIDTH(32)) shown (
        .source_clock(circuit_clock), .update(refresh), .value(glyphs),
        .destination_clock(board_clock), .updated(), .crossed(shown_glyphs)
    );
    circuit_system system (
        .clock(circuit_clock), .serial_clock(board_clock), .serial_receive(serial_receive),
        .clear_fault(clear_fault), .serial_transmit(serial_transmit), .ticking(ticking),
        .halted(halted), .tick_count(tick_count), .fault_code(fault_code)
    );
    push_button center (.clock(circuit_clock), .button(button_center), .pressed(clear_fault));
    push_button left (.clock(circuit_clock), .button(button_left), .pressed(previous_view));
    push_button right (.clock(circuit_clock), .button(button_right), .pressed(next_view));
    decimal_digits converter (
        .clock(circuit_clock), .start(refresh || previous_view || next_view),
        .value(shown_value), .digits(digits)
    );
    number_glyphs formatter (.digits(digits), .glyphs(number));
endmodule
