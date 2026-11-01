// Frames are byte-stuffed (0xc0 ends a frame, 0xdb escapes) and end with a CRC-16 of their content.
// A request is [sequence, command, ...]; a response is [sequence, status, tick count, payload].
// STATE_BYTES is zero in a build that reads back only the input and output nodes.
module packet_controller #(
    parameter NODE_COUNT = 1,
    parameter OUTPUT_COUNT = 0,
    parameter STATE_BYTES = 1,
    parameter OUTPUT_BYTES = 1,
    parameter [63:0] CIRCUIT_ID = 0,
    parameter MAX_INPUTS = 256
) (
    input wire clock,
    input wire [7:0] receive_data,
    input wire receive_ready,
    input wire receive_error,
    output reg [7:0] transmit_data = 0,
    output reg transmit_start = 0,
    input wire transmit_busy,
    input wire clear_fault,
    output wire ticking,
    output wire waiting,
    output wire [63:0] tick_count,
    output reg [3:0] fault_code = 0
);
    localparam [7:0] PROTOCOL_VERSION = 2;
    localparam [7:0] FRAME_END = 8'hc0, ESCAPE = 8'hdb, ESCAPED_END = 8'hdc, ESCAPED_ESCAPE = 8'hdd;
    localparam [7:0] IDENTIFY = 1, ADVANCE = 2;
    localparam [7:0] NO_REPORT = 0, OUTPUTS_REPORT = 1, STATES_REPORT = 2;
    localparam [3:0] FRAMING_FAULT = 1, CHECKSUM_FAULT = 2, MALFORMED_FAULT = 3, COMMAND_FAULT = 4,
        INPUT_FAULT = 5, TARGET_FAULT = 6, REPORT_FAULT = 7;
    localparam [2:0] RECEIVE = 0, LOAD_INPUT = 1, FETCH_INPUT = 2, SET_INPUT = 3, RUN = 4,
        TRANSMIT = 5, TRANSMIT_ACCEPT = 6, TRANSMIT_COMPLETE = 7;
    localparam HEADER_BYTES = 10, ADVANCE_BYTES = 11, IDENTITY_BYTES = 27, CHECKSUM_BYTES = 2;
    localparam INPUT_INDEX_WIDTH = MAX_INPUTS > 1 ? $clog2(MAX_INPUTS) : 1;
    localparam REQUEST_MAX_BYTES = ADVANCE_BYTES + 4 * MAX_INPUTS + CHECKSUM_BYTES;
    localparam REQUEST_POSITION_WIDTH = $clog2(REQUEST_MAX_BYTES + 2);
    localparam READBACK_BYTES = STATE_BYTES + OUTPUT_BYTES;
    localparam READBACK_INDEX_WIDTH = $clog2(64'd8 * READBACK_BYTES);
    localparam PAYLOAD_MAX_BYTES = STATE_BYTES > OUTPUT_BYTES
        ? (STATE_BYTES > IDENTITY_BYTES ? STATE_BYTES : IDENTITY_BYTES)
        : (OUTPUT_BYTES > IDENTITY_BYTES ? OUTPUT_BYTES : IDENTITY_BYTES);
    localparam RESPONSE_MAX_BYTES = HEADER_BYTES + PAYLOAD_MAX_BYTES + CHECKSUM_BYTES + 1;
    localparam RESPONSE_POSITION_WIDTH = $clog2(RESPONSE_MAX_BYTES + 1);
    localparam [31:0] IDENTITY_NODE_COUNT = NODE_COUNT, IDENTITY_OUTPUT_COUNT = OUTPUT_COUNT,
        IDENTITY_STATE_BYTES = STATE_BYTES, IDENTITY_OUTPUT_BYTES = OUTPUT_BYTES;
    localparam [15:0] IDENTITY_MAX_INPUTS = MAX_INPUTS;
    localparam [IDENTITY_BYTES * 8 - 1:0] IDENTITY = {
        IDENTITY_MAX_INPUTS, IDENTITY_OUTPUT_BYTES, IDENTITY_STATE_BYTES,
        IDENTITY_OUTPUT_COUNT, IDENTITY_NODE_COUNT, CIRCUIT_ID, PROTOCOL_VERSION
    };

    reg [2:0] phase = RECEIVE;
    reg escaping = 0;
    reg malformed = 0;
    reg invalid_input = 0;
    reg [REQUEST_POSITION_WIDTH-1:0] request_length = 0;
    reg [15:0] request_crc = 16'hffff;
    reg [7:0] request_sequence = 0;
    reg [7:0] request_command = 0;
    reg [63:0] request_target = 0;
    reg [7:0] request_report = 0;
    reg [23:0] input_bytes = 0;
    reg [31:0] inputs [0:MAX_INPUTS-1];
    reg [31:0] input_word = 0;
    reg [INPUT_INDEX_WIDTH:0] input_count = 0;
    reg [INPUT_INDEX_WIDTH:0] applied_inputs = 0;
    reg set_input = 0;
    reg [63:0] tick_limit = 0;
    reg [7:0] response_status = 0;
    reg [RESPONSE_POSITION_WIDTH-1:0] response_position = 0;
    reg [RESPONSE_POSITION_WIDTH-1:0] payload_bytes = 0;
    reg [15:0] response_crc = 16'hffff;
    reg escape_sent = 0;
    wire engine_busy;
    wire [READBACK_BYTES * 8 - 1:0] readback;

    wire [7:0] received_byte = !escaping ? receive_data
        : receive_data == ESCAPED_END ? FRAME_END
        : ESCAPE;
    wire receiving_content = phase == RECEIVE && receive_ready && receive_data != FRAME_END
        && !(receive_data == ESCAPE && !escaping);
    wire [REQUEST_POSITION_WIDTH-1:0] input_position = request_length - ADVANCE_BYTES;
    wire [REQUEST_POSITION_WIDTH-1:0] request_input_bytes =
        request_length - ADVANCE_BYTES - CHECKSUM_BYTES;
    wire [31:0] received_input = {received_byte, input_bytes};
    wire frame_valid = !malformed && request_crc == 0 && request_length >= 2 + CHECKSUM_BYTES;
    wire identify_valid = request_length == 2 + CHECKSUM_BYTES;
    wire advance_valid = request_length >= ADVANCE_BYTES + CHECKSUM_BYTES
        && request_input_bytes[1:0] == 0;

    wire [RESPONSE_POSITION_WIDTH-1:0] checksum_position = HEADER_BYTES + payload_bytes;
    wire [RESPONSE_POSITION_WIDTH-1:0] payload_position = response_position - HEADER_BYTES;
    wire [READBACK_INDEX_WIDTH-1:0] readback_bit_position =
        (payload_position + (request_report == OUTPUTS_REPORT ? STATE_BYTES : 0)) << 3;
    reg [7:0] response_byte;
    wire special_byte = response_byte == FRAME_END || response_byte == ESCAPE;
    wire frame_finished = response_position == checksum_position + CHECKSUM_BYTES;

    assign ticking = engine_busy;
    assign waiting = phase == RECEIVE;

    compiled_circuit circuit (
        .clock(clock), .set_input(set_input),
        .input_index({1'b0, input_word[30:0]}), .input_value(input_word[31]),
        .run(phase == RUN), .tick_limit(tick_limit), .busy(engine_busy), .tick_count(tick_count),
        .readback(readback)
    );

    function [15:0] crc16;
        input [15:0] previous;
        input [7:0] data;
        integer bit_index;
        reg [15:0] value;
        begin
            value = previous ^ {data, 8'b0};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                value = value[15] ? (value << 1) ^ 16'h1021 : value << 1;
            crc16 = value;
        end
    endfunction

    task report_fault(input [3:0] code);
        if (fault_code == 0)
            fault_code <= code;
    endtask

    task respond(input [7:0] status, input [RESPONSE_POSITION_WIDTH-1:0] bytes);
        begin
            response_status <= status;
            payload_bytes <= bytes;
            response_position <= 0;
            response_crc <= 16'hffff;
            escape_sent <= 0;
            phase <= TRANSMIT;
        end
    endtask

    task reject(input [3:0] code);
        begin
            report_fault(code);
            respond(code, 0);
        end
    endtask

    always @* begin
        if (response_position == 0)
            response_byte = request_sequence;
        else if (response_position == 1)
            response_byte = response_status;
        else if (response_position < HEADER_BYTES)
            response_byte = tick_count[(response_position - 2) * 8 +: 8];
        else if (response_position < checksum_position)
            response_byte = request_command == IDENTIFY
                ? IDENTITY[payload_position * 8 +: 8]
                : readback[readback_bit_position +: 8];
        else if (response_position == checksum_position)
            response_byte = response_crc[15:8];
        else
            response_byte = response_crc[7:0];
    end

    always @(posedge clock)
        input_word <= inputs[applied_inputs[INPUT_INDEX_WIDTH-1:0]];

    always @(posedge clock)
        if (receiving_content && request_length >= ADVANCE_BYTES && input_position[1:0] == 3
            && input_position[REQUEST_POSITION_WIDTH-1:2] < MAX_INPUTS)
            inputs[input_position[INPUT_INDEX_WIDTH+1:2]] <= received_input;

    always @(posedge clock) begin
        set_input <= 0;
        transmit_start <= 0;
        if (clear_fault)
            fault_code <= 0;
        if (receive_error) begin
            report_fault(FRAMING_FAULT);
            malformed <= 1;
        end
        case (phase)
            RECEIVE: if (receive_ready) begin
                if (receive_data == FRAME_END) begin
                    escaping <= 0;
                    malformed <= 0;
                    invalid_input <= 0;
                    request_length <= 0;
                    request_crc <= 16'hffff;
                    if (request_length == 0) begin
                    end else if (!frame_valid) begin
                        report_fault(malformed ? MALFORMED_FAULT : CHECKSUM_FAULT);
                    end else if (request_command == IDENTIFY && identify_valid) begin
                        respond(0, IDENTITY_BYTES);
                    end else if (request_command == ADVANCE && advance_valid) begin
                        if (invalid_input)
                            reject(INPUT_FAULT);
                        else if (request_target < tick_count)
                            reject(TARGET_FAULT);
                        else if (request_report > STATES_REPORT
                            || request_report == STATES_REPORT && STATE_BYTES == 0)
                            reject(REPORT_FAULT);
                        else begin
                            tick_limit <= request_target;
                            input_count <= request_input_bytes[REQUEST_POSITION_WIDTH-1:2];
                            applied_inputs <= 0;
                            phase <= LOAD_INPUT;
                        end
                    end else if (request_command == IDENTIFY || request_command == ADVANCE) begin
                        report_fault(MALFORMED_FAULT);
                    end else begin
                        reject(COMMAND_FAULT);
                    end
                end else if (receive_data == ESCAPE && !escaping) begin
                    escaping <= 1;
                end else begin
                    escaping <= 0;
                    if (escaping && receive_data != ESCAPED_END && receive_data != ESCAPED_ESCAPE)
                        malformed <= 1;
                    if (request_length == REQUEST_MAX_BYTES)
                        malformed <= 1;
                    else
                        request_length <= request_length + 1;
                    request_crc <= crc16(request_crc, received_byte);
                    if (request_length == 0)
                        request_sequence <= received_byte;
                    else if (request_length == 1)
                        request_command <= received_byte;
                    else if (request_length < ADVANCE_BYTES - 1)
                        request_target[(request_length - 2) * 8 +: 8] <= received_byte;
                    else if (request_length == ADVANCE_BYTES - 1)
                        request_report <= received_byte;
                    else if (input_position[1:0] != 3)
                        input_bytes[input_position[1:0] * 8 +: 8] <= received_byte;
                    else if (received_input[30:0] >= NODE_COUNT)
                        invalid_input <= 1;
                end
            end
            LOAD_INPUT: phase <= applied_inputs == input_count ? RUN : FETCH_INPUT;
            FETCH_INPUT: begin
                set_input <= 1;
                phase <= SET_INPUT;
            end
            SET_INPUT: begin
                applied_inputs <= applied_inputs + 1;
                phase <= LOAD_INPUT;
            end
            RUN: if (!engine_busy && tick_count == tick_limit)
                respond(0, request_report == STATES_REPORT ? STATE_BYTES
                    : request_report == OUTPUTS_REPORT ? OUTPUT_BYTES
                    : 0);
            TRANSMIT: begin
                transmit_start <= 1;
                phase <= TRANSMIT_ACCEPT;
                if (frame_finished)
                    transmit_data <= FRAME_END;
                else if (special_byte && !escape_sent)
                    transmit_data <= ESCAPE;
                else if (special_byte)
                    transmit_data <= response_byte == FRAME_END ? ESCAPED_END : ESCAPED_ESCAPE;
                else
                    transmit_data <= response_byte;
            end
            TRANSMIT_ACCEPT: if (transmit_busy)
                phase <= TRANSMIT_COMPLETE;
            TRANSMIT_COMPLETE: if (!transmit_busy) begin
                if (frame_finished) begin
                    phase <= RECEIVE;
                end else if (special_byte && !escape_sent) begin
                    escape_sent <= 1;
                    phase <= TRANSMIT;
                end else begin
                    escape_sent <= 0;
                    if (response_position < checksum_position)
                        response_crc <= crc16(response_crc, response_byte);
                    response_position <= response_position + 1;
                    phase <= TRANSMIT;
                end
            end
        endcase
    end
endmodule
