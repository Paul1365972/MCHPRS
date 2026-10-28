// Answers the host's requests: one frame in, one frame out.
//
// The circuit's ports are on `circuit_clock`, everything else is on `board_clock`. A request to the
// circuit holds its ticks from the tick boundary at which it applies until its reply is sent, so
// the reply reads the tick count and the state straight from the circuit's registers.
//
// An answer is the status byte, then the payload of the request, sent from the lowest byte up:
// - HELLO: the build nonce.
// - RUN, DRIVE and READ: the tick count, then the shown state (DRIVE and READ with READ_SHOWN),
//   the whole state (READ with READ_ALL) or nothing (RUN), padded to whole bytes.
module controller (
    input logic board_clock,
    input logic received_valid,
    input logic [7:0] received,
    output logic transmit_valid,
    output logic [7:0] transmit,
    input logic transmit_ready,

    input logic circuit_clock,
    output logic hold,
    output logic run_start,
    output pkg::mode_t run_mode,
    output logic [pkg::COUNT_BITS-1:0] run_count,
    input logic idle,
    input logic [63:0] tick_count,
    output logic drive_valid,
    output logic [23:0] drive_node,
    output logic [5:0] drive_input,
    output logic shown_read,
    input logic [circuit_pkg::STATE_BITS-1:0] state
);
    localparam int TICK_COUNT_BYTES = 8;
    localparam int STATE_BYTES = (circuit_pkg::STATE_BITS + 7) / 8;
    localparam int SHOWN_BYTES = (circuit_pkg::SHOWN_BITS + 7) / 8;
    localparam int REPLY_BYTES = TICK_COUNT_BYTES + STATE_BYTES;
    localparam int POSITION_BITS = $clog2(REPLY_BYTES + 1);

    typedef struct packed {
        logic [31:0] build_nonce;
        pkg::status_t status;
    } hello_answer_t;
    localparam int ANSWER_BYTES = 1 + 4;

    typedef enum logic [2:0] {
        // Waiting for a request.
        IDLE,
        // Waiting until the circuit holds its ticks and has applied the request.
        CROSS,
        // Sending the status byte and the rest of a short answer.
        ANSWER,
        // Sending the tick count and the state.
        REPLY,
        // Waiting until the circuit runs again.
        RELEASE,
        // Sending the checksum and the closing FRAME_END.
        CLOSE
    } step_t;

    logic frame_completed;
    pkg::status_t frame_status;
    logic [7:0] frame_opcode;
    logic [pkg::ARGUMENT_BYTES*8-1:0] frame_argument;
    pkg::status_t status;
    step_t first_step;
    logic [2:0] answer_bytes;
    logic [POSITION_BITS-1:0] reply_bytes;

    step_t step = IDLE;
    logic [7:0] opcode = 0;
    logic [pkg::RUN_BITS-1:0] argument = 0;
    logic [ANSWER_BYTES*8-1:0] answer = 0;
    logic [2:0] answer_bytes_left = 0;
    logic [POSITION_BITS-1:0] reply_bytes_left = 0;
    logic [POSITION_BITS-1:0] position = 0;
    logic [REPLY_BYTES*8-1:0] reply;
    logic [7:0] reply_byte = 0;
    logic reply_byte_valid = 0;
    logic request = 0;
    logic acknowledged;
    logic apply;
    logic let_go;
    pkg::run_t run;
    pkg::drive_t drive;
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
    assign content = step == REPLY ? reply_byte : answer[7:0];
    assign content_valid = step == ANSWER || (step == REPLY && reply_byte_valid);
    assign content_taken = content_valid && content_ready;
    assign closing = step == CLOSE;

    assign hello.build_nonce = circuit_pkg::BUILD_NONCE;
    assign hello.status = status;

    always_comb begin
        status = pkg::OK;
        first_step = ANSWER;
        answer_bytes = 1;
        reply_bytes = POSITION_BITS'(TICK_COUNT_BYTES);
        if (frame_status != pkg::OK) begin
            status = frame_status;
        end else begin
            unique case (frame_opcode)
                pkg::HELLO: answer_bytes = 3'(ANSWER_BYTES);
                pkg::RUN, pkg::DRIVE, pkg::READ: begin
                    first_step = CROSS;
                    if (frame_opcode == pkg::READ && frame_argument == pkg::READ_ALL)
                        reply_bytes = POSITION_BITS'(TICK_COUNT_BYTES + STATE_BYTES);
                    else if (frame_opcode != pkg::RUN)
                        reply_bytes = POSITION_BITS'(TICK_COUNT_BYTES + SHOWN_BYTES);
                end
                default: status = pkg::UNKNOWN_OPCODE;
            endcase
        end
    end

    always_ff @(posedge board_clock) begin
        reply_byte <= reply[position*8 +: 8];
        reply_byte_valid <= step == REPLY && !content_taken;
        unique case (step)
            IDLE: if (frame_completed) begin
                opcode <= frame_opcode;
                argument <= frame_argument[pkg::RUN_BITS-1:0];
                answer <= hello;
                answer_bytes_left <= answer_bytes;
                reply_bytes_left <= reply_bytes;
                position <= 0;
                request <= first_step == CROSS;
                step <= first_step;
            end
            CROSS: if (acknowledged) step <= ANSWER;
            ANSWER: if (content_taken) begin
                answer <= answer >> 8;
                answer_bytes_left <= answer_bytes_left - 1'b1;
                if (answer_bytes_left == 1) step <= request ? REPLY : CLOSE;
            end
            REPLY: if (content_taken) begin
                position <= position + 1'b1;
                reply_bytes_left <= reply_bytes_left - 1'b1;
                if (reply_bytes_left == 1) begin
                    request <= 0;
                    step <= RELEASE;
                end
            end
            RELEASE: if (!acknowledged) step <= CLOSE;
            CLOSE: if (closed) step <= IDLE;
            default: step <= IDLE;
        endcase
    end
endmodule
