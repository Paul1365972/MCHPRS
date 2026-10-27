// The circuit's ports are on `circuit_clock`,
// and so is the choice of a reply byte among the tick count and the state.
// Everything else is on `board_clock`, where nothing grows with the circuit.
// A request to the circuit holds its ticks from the tick boundary at which it applies until its reply is sent,
// so the tick count and the state hold still while the reply reads them.
module controller (
    input logic board_clock,
    input logic received_valid,
    input logic [7:0] received,
    output logic transmit_valid,
    output logic [7:0] transmit,
    input logic transmit_ready,

    output logic pll_enable = 0,
    output logic pll_write,
    output logic [6:0] pll_address,
    output logic [15:0] pll_write_data,
    input logic pll_ready,
    input logic [15:0] pll_read_data,
    input logic pll_locked,
    output logic pll_reset = 1,
    output logic circuit_clock_enable,

    input logic circuit_clock,
    output logic hold,
    output logic run_start,
    output pkg::mode_t run_mode,
    output logic [pkg::COUNT_BITS-1:0] run_count,
    input logic idle,
    input logic [63:0] tick_count,
    output logic drive_valid,
    output logic [pkg::NODE_BITS-1:0] drive_node,
    output pkg::drive_input_t drive_input,
    output logic shown_read,
    input logic [circuit_pkg::STATE_BITS-1:0] state,

    output logic [15:0] ticks_run = 0,
    output pkg::status_t last_status = pkg::OK
);
    localparam int TICK_COUNT_BYTES = 8;
    localparam int STATE_BYTES = (circuit_pkg::STATE_BITS + 7) / 8;
    localparam int SHOWN_BYTES = (circuit_pkg::SHOWN_BITS + 7) / 8;
    localparam int REPLY_BYTES = TICK_COUNT_BYTES + STATE_BYTES;
    localparam int POSITION_BITS = $clog2(REPLY_BYTES + 1);
    // The clocks `position` takes through `fetch_position` into `fetched_byte`.
    localparam int REPLY_FETCH_CLOCKS = 2;

    typedef struct packed {
        logic [6:0] unused;
        logic pll_locked;
        logic [31:0] build_nonce;
        pkg::status_t status;
    } hello_answer_t;
    localparam int ANSWER_BYTES = 1 + 4 + 1;
    localparam int PLL_READ_ANSWER_BYTES = 3;

    typedef enum logic [2:0] {
        AWAIT_REQUEST,
        AWAIT_CIRCUIT,
        AWAIT_PLL,
        SEND_ANSWER,
        SEND_REPLY,
        AWAIT_RELEASE,
        SEND_CLOSE
    } step_t;

    logic frame_completed;
    pkg::status_t frame_status;
    logic [7:0] frame_opcode;
    logic [pkg::ARGUMENT_BYTES*8-1:0] frame_argument;
    pkg::status_t status;
    step_t first_step;
    logic [2:0] answer_bytes;
    logic [POSITION_BITS-1:0] reply_bytes;

    step_t step = AWAIT_REQUEST;
    logic [7:0] opcode = 0;
    logic [pkg::RUN_BITS-1:0] argument = 0;
    logic [ANSWER_BYTES*8-1:0] answer = 0;
    logic [2:0] answer_bytes_left = 0;
    logic [POSITION_BITS-1:0] reply_bytes_left = 0;
    logic [POSITION_BITS-1:0] position = 0;
    logic [REPLY_BYTES*8-1:0] reply;
    logic [7:0] reply_byte = 0;
    logic reply_byte_valid = 0;
    // A reply byte crosses as `request` does: `fetch` asks for the byte at `position`,
    // which holds still while `fetch` is high, and `fetched` says that `fetched_byte` is that byte.
    logic fetch = 0;
    logic [1:0] fetch_crossing = 0;
    logic [REPLY_FETCH_CLOCKS-1:0] fetch_age = 0;
    logic [POSITION_BITS-1:0] fetch_position = 0;
    logic [7:0] fetched_byte = 0;
    logic fetched = 0;
    logic [1:0] fetched_crossing = 0;
    logic [1:0] locked_crossing = 0;
    logic locked;
    logic stopped;
    logic request = 0;
    logic acknowledged;
    logic apply;
    logic let_go;
    pkg::run_t run;
    pkg::drive_t drive;
    pkg::pll_access_t pll_access;
    logic content_valid;
    logic [7:0] content;
    logic content_ready;
    logic content_taken;
    logic closing;
    logic closed;
    hello_answer_t hello;

    frame_rx receiver (
        .clock(board_clock), .valid(received_valid), .data(received),
        .completed(frame_completed), .status(frame_status), .content({frame_argument, frame_opcode})
    );
    frame_tx transmitter (
        .clock(board_clock), .content_valid(content_valid), .content(content),
        .content_ready(content_ready), .close(closing), .closed(closed),
        .transmit_valid(transmit_valid), .transmit(transmit), .transmit_ready(transmit_ready)
    );
    handshake crossing (
        .board_clock(board_clock), .request(request), .acknowledged(acknowledged),
        .circuit_clock(circuit_clock), .idle(idle), .hold(hold), .apply(apply), .let_go(let_go)
    );

    assign locked = locked_crossing[1];
    assign stopped = pll_reset || !locked;
    assign circuit_clock_enable = !stopped;

    assign pll_access = argument[pkg::PLL_ACCESS_BITS-1:0];
    assign pll_write = opcode == pkg::PLL_WRITE;
    assign pll_address = pll_access.address;
    assign pll_write_data = pll_access.data;

    assign run = argument;
    assign run_start = apply && opcode == pkg::RUN;
    assign run_mode = run.mode;
    assign run_count = run.count;
    assign drive = argument[pkg::DRIVE_BITS-1:0];
    assign drive_valid = apply && opcode == pkg::DRIVE;
    assign drive_node = drive.node;
    assign drive_input = {drive.kind, drive.value};
    assign shown_read = let_go && opcode != pkg::RUN;

    assign reply = {(STATE_BYTES*8)'(state), tick_count};
    assign content = step == SEND_REPLY ? reply_byte : answer[7:0];
    assign content_valid = step == SEND_ANSWER || (step == SEND_REPLY && reply_byte_valid);
    assign content_taken = content_valid && content_ready;
    assign closing = step == SEND_CLOSE;

    assign hello.unused = 0;
    assign hello.pll_locked = locked;
    assign hello.build_nonce = circuit_pkg::BUILD_NONCE;
    assign hello.status = status;

    always_comb begin
        status = pkg::OK;
        first_step = SEND_ANSWER;
        answer_bytes = 1;
        reply_bytes = POSITION_BITS'(TICK_COUNT_BYTES);
        if (frame_status != pkg::OK) begin
            status = frame_status;
        end else begin
            unique case (frame_opcode)
                pkg::HELLO: answer_bytes = 3'(ANSWER_BYTES);
                pkg::RUN, pkg::DRIVE, pkg::READ: begin
                    if (stopped) status = pkg::CLOCK_STOPPED;
                    else first_step = AWAIT_CIRCUIT;
                    if (frame_opcode == pkg::READ && frame_argument == pkg::READ_ALL)
                        reply_bytes = POSITION_BITS'(TICK_COUNT_BYTES + STATE_BYTES);
                    else if (frame_opcode != pkg::RUN)
                        reply_bytes = POSITION_BITS'(TICK_COUNT_BYTES + SHOWN_BYTES);
                end
                pkg::PLL_WRITE: first_step = AWAIT_PLL;
                pkg::PLL_READ: begin
                    first_step = AWAIT_PLL;
                    answer_bytes = 3'(PLL_READ_ANSWER_BYTES);
                end
                pkg::RELEASE_CLOCK: begin end
                default: status = pkg::UNKNOWN_OPCODE;
            endcase
        end
    end

    always_ff @(posedge board_clock) begin
        locked_crossing <= {locked_crossing[0], pll_locked};
        pll_enable <= 0;
        fetched_crossing <= {fetched_crossing[0], fetched};
        if (fetch && fetched_crossing[1]) begin
            reply_byte <= fetched_byte;
            reply_byte_valid <= 1;
            fetch <= 0;
        end else if (step == SEND_REPLY && !reply_byte_valid && !fetch && !fetched_crossing[1]) begin
            fetch <= 1;
        end
        if (content_taken) reply_byte_valid <= 0;
        unique case (step)
            AWAIT_REQUEST: if (frame_completed) begin
                opcode <= frame_opcode;
                argument <= frame_argument[pkg::RUN_BITS-1:0];
                answer <= hello;
                last_status <= status;
                answer_bytes_left <= answer_bytes;
                reply_bytes_left <= reply_bytes;
                position <= 0;
                request <= first_step == AWAIT_CIRCUIT;
                pll_enable <= first_step == AWAIT_PLL;
                if (status == pkg::OK && frame_opcode == pkg::RELEASE_CLOCK) pll_reset <= 0;
                step <= first_step;
            end
            AWAIT_CIRCUIT: if (acknowledged) begin
                ticks_run <= tick_count[15:0];
                step <= SEND_ANSWER;
            end
            AWAIT_PLL: if (pll_ready) begin
                answer[8 +: 16] <= pll_read_data;
                step <= SEND_ANSWER;
            end
            SEND_ANSWER: if (content_taken) begin
                answer <= answer >> 8;
                answer_bytes_left <= answer_bytes_left - 1'b1;
                if (answer_bytes_left == 1) step <= request ? SEND_REPLY : SEND_CLOSE;
            end
            SEND_REPLY: if (content_taken) begin
                position <= position + 1'b1;
                reply_bytes_left <= reply_bytes_left - 1'b1;
                if (reply_bytes_left == 1) begin
                    request <= 0;
                    step <= AWAIT_RELEASE;
                end
            end
            AWAIT_RELEASE: if (!acknowledged) step <= SEND_CLOSE;
            SEND_CLOSE: if (closed) step <= AWAIT_REQUEST;
            default: step <= AWAIT_REQUEST;
        endcase
    end

    always_ff @(posedge circuit_clock) begin
        fetch_crossing <= {fetch_crossing[0], fetch};
        fetch_age <= fetch_crossing[1] ? {fetch_age[REPLY_FETCH_CLOCKS-2:0], 1'b1} : '0;
        fetch_position <= position;
        fetched_byte <= reply[fetch_position*8 +: 8];
        fetched <= fetch_crossing[1] && fetch_age[REPLY_FETCH_CLOCKS-1];
    end
endmodule
