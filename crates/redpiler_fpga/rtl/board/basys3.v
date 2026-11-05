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
    output wire decimal_point,
    output wire [3:0] vga_red,
    output wire [3:0] vga_green,
    output wire [3:0] vga_blue,
    output wire vga_horizontal_sync,
    output wire vga_vertical_sync
);
    localparam CLOCKS_PER_SECOND = 100_000_000;
    localparam CLOCKS_PER_REFRESH = CLOCKS_PER_SECOND / 10;
    localparam ACTIVITY_CLOCKS = CLOCKS_PER_SECOND / 20;
    localparam [1:0] TICK_RATE_VIEW = 0, TICK_COUNT_VIEW = 1, FAULT_VIEW = 2;
    localparam [7:0] DASH = 8'h40, LETTER_E = 8'h79;

    wire ticking, halted, clear_fault, previous_view, next_view;
    wire ticking_shown, receiving, transmitting;
    wire [3:0] fault_code;
    wire [63:0] tick_count;
    wire [79:0] digits;
    wire [31:0] number;
    reg [26:0] heartbeat = 0;
    reg [1:0] view = TICK_RATE_VIEW;
    reg [26:0] second_clocks = 0;
    reg [63:0] second_start = 0;
    reg [63:0] tick_rate = 0;
    reg [23:0] refresh_clocks = 0;
    wire refresh = refresh_clocks == CLOCKS_PER_REFRESH - 1;
    wire [63:0] shown_value = view == TICK_RATE_VIEW ? tick_rate
        : view == TICK_COUNT_VIEW ? tick_count
        : fault_code;
    wire [31:0] glyphs = view != FAULT_VIEW ? number
        : fault_code == 0 ? {4{DASH}}
        : {LETTER_E, number[23:0]};

    always @(posedge clock) begin
        heartbeat <= heartbeat + 1;
        refresh_clocks <= refresh ? 0 : refresh_clocks + 1;
        if (second_clocks == CLOCKS_PER_SECOND - 1) begin
            second_clocks <= 0;
            second_start <= tick_count;
            tick_rate <= tick_count - second_start;
        end else begin
            second_clocks <= second_clocks + 1;
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

    circuit_system system (
        .clock(clock), .serial_receive(serial_receive), .clear_fault(clear_fault),
        .serial_transmit(serial_transmit), .ticking(ticking), .halted(halted),
        .tick_count(tick_count), .fault_code(fault_code),
        .vga_color({vga_red, vga_green, vga_blue}), .vga_horizontal_sync(vga_horizontal_sync),
        .vga_vertical_sync(vga_vertical_sync)
    );
    push_button center (.clock(clock), .button(button_center), .pressed(clear_fault));
    push_button left (.clock(clock), .button(button_left), .pressed(previous_view));
    push_button right (.clock(clock), .button(button_right), .pressed(next_view));
    activity #(.HOLD_CLOCKS(ACTIVITY_CLOCKS)) ticking_activity (
        .clock(clock), .signal(ticking), .active(ticking_shown)
    );
    activity #(.HOLD_CLOCKS(ACTIVITY_CLOCKS)) receive_activity (
        .clock(clock), .signal(!serial_receive), .active(receiving)
    );
    activity #(.HOLD_CLOCKS(ACTIVITY_CLOCKS)) transmit_activity (
        .clock(clock), .signal(!serial_transmit), .active(transmitting)
    );
    decimal_digits converter (
        .clock(clock), .start(refresh || previous_view || next_view),
        .value(shown_value), .digits(digits)
    );
    number_glyphs formatter (.digits(digits), .glyphs(number));
    seven_segment_display display (
        .clock(clock), .glyphs(glyphs), .segment(segment), .anode(anode),
        .decimal_point(decimal_point)
    );
endmodule
