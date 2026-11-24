// Wire constants shared with `src/protocol.rs`.
package pkg;
    // The framing bytes of SLIP (RFC 1055).
    localparam logic [7:0] FRAME_END = 8'hc0;
    localparam logic [7:0] ESCAPE = 8'hdb;
    localparam logic [7:0] ESCAPED_END = 8'hdc;
    localparam logic [7:0] ESCAPED_ESCAPE = 8'hdd;

    typedef enum logic [7:0] {
        HELLO = 8'd0,
        RUN = 8'd1,
        DRIVE = 8'd2,
        READ = 8'd3,
        PLL_WRITE = 8'd4,
        PLL_READ = 8'd5,
        RELEASE_CLOCK = 8'd6
    } opcode_t;

    typedef enum logic [7:0] {
        OK = 8'd0,
        BAD_LENGTH = 8'd1,
        BAD_ESCAPE = 8'd2,
        UNKNOWN_OPCODE = 8'd3,
        CLOCK_STOPPED = 8'd4
    } status_t;

    localparam int ARGUMENT_BYTES = 6;

    localparam logic [ARGUMENT_BYTES*8-1:0] READ_SHOWN = 0;
    localparam logic [ARGUMENT_BYTES*8-1:0] READ_ALL = 1;

    typedef enum logic [1:0] {
        PAUSED = 2'd0,
        UNLIMITED = 2'd1,
        COUNTED = 2'd2,
        PERIODIC = 2'd3
    } mode_t;

    localparam int COUNT_BITS = 40;

    typedef struct packed {
        mode_t mode;
        logic [COUNT_BITS-1:0] count;
    } run_t;
    localparam int RUN_BITS = 2 + COUNT_BITS;

    typedef enum logic [1:0] {
        INPUT_SET = 2'd0,
        INPUT_TOGGLE = 2'd1,
        INPUT_PRESS = 2'd2,
        INPUT_TOUCH = 2'd3
    } input_kind_t;

    typedef struct packed {
        input_kind_t kind;
        // The strength of INPUT_SET.
        // Its lowest bit says whether INPUT_TOUCH touches.
        logic [3:0] value;
    } drive_input_t;

    localparam int NODE_BITS = 24;

    typedef struct packed {
        input_kind_t kind;
        logic [3:0] value;
        logic [NODE_BITS-1:0] node;
    } drive_t;
    localparam int DRIVE_BITS = 2 + 4 + NODE_BITS;

    typedef struct packed {
        logic [6:0] address;
        logic [15:0] data;
    } pll_access_t;
    localparam int PLL_ACCESS_BITS = 7 + 16;

    // When a node fires, numbered as `analysis::Firing`.
    localparam int FIRES_WHEN_DUE = 0;
    localparam int FIRES_IN_PRIORITY = 1;
    localparam int FIRES_ON_TICK_START_MISMATCH = 2;

    function automatic logic [3:0] attenuated(logic [3:0] strength, logic [3:0] weight);
        attenuated = strength > weight ? strength - weight : '0;
    endfunction

    function automatic logic [3:0] maximum(logic [3:0] first, logic [3:0] second);
        maximum = first > second ? first : second;
    endfunction

    function automatic logic [3:0] comparator_rear(
        logic far_valid, logic [3:0] far, logic [3:0] default_strength
    );
        comparator_rear = far_valid && default_strength != 4'd15 ? far : default_strength;
    endfunction

    function automatic logic [3:0] comparator_output(
        logic subtract, logic [3:0] rear, logic [3:0] side_strength
    );
        if (rear < side_strength) comparator_output = '0;
        else comparator_output = subtract ? rear - side_strength : rear;
    endfunction

    function automatic logic repeater_ticked(logic powered, logic default_on);
        repeater_ticked = !powered || default_on;
    endfunction

    function automatic logic repeater_pulses(logic powered, logic default_on);
        repeater_pulses = !powered && !default_on;
    endfunction
endpackage
