// STATE_BYTES is zero in a build that reads back only the input and output nodes.
module packet_controller #(
    parameter NODE_COUNT = 1,
    parameter OUTPUT_COUNT = 0,
    parameter STATE_BYTES = 1,
    parameter OUTPUT_BYTES = 1,
    parameter [63:0] CIRCUIT_ID = 0
) (
    input wire clock,
    input wire [7:0] receive_data,
    input wire receive_ready,
    input wire receive_error,
    output wire receive_accept,
    output reg [7:0] transmit_data = 0,
    output reg transmit_start = 0,
    input wire transmit_busy,
    output wire busy,
    output wire [63:0] tick_count,
    output reg fault = 0
);
    localparam FIND_FIRST = 0, FIND_SECOND = 1, RECEIVE = 2,
        VALIDATE = 3, BOUNDARY = 4, DISPATCH = 5, WAIT_COMPLETE = 6,
        CAPTURE = 7, TRANSMIT = 8, TRANSMIT_ACCEPT = 9, TRANSMIT_COMPLETE = 10;
    localparam [31:0] RESPONSE_NODE_COUNT = NODE_COUNT;
    localparam [31:0] RESPONSE_OUTPUT_COUNT = OUTPUT_COUNT;
    localparam READBACK_BYTES = STATE_BYTES + OUTPUT_BYTES;
    localparam READBACK_INDEX_WIDTH = $clog2(64'd8 * READBACK_BYTES);
    localparam RESPONSE_MAX_BYTES = 64'd18 + (STATE_BYTES > OUTPUT_BYTES ? STATE_BYTES : OUTPUT_BYTES);
    localparam RESPONSE_POSITION_WIDTH = $clog2(RESPONSE_MAX_BYTES);
    localparam RESPONSE_LENGTH_WIDTH = $clog2(RESPONSE_MAX_BYTES + 1);

    reg [3:0] phase = FIND_FIRST;
    reg [3:0] request_position = 0;
    reg [95:0] request = 0;
    reg [15:0] request_crc = 16'hffff;
    reg [15:0] received_crc = 0;
    wire [7:0] request_sequence = request[7:0];
    wire [7:0] request_opcode = request[15:8];
    wire [63:0] request_argument = request[79:16];
    wire compact_response = request_opcode >= 5 && request_opcode <= 8;
    reg [7:0] response_opcode = 0;
    reg [111:0] response_header = 0;
    reg [15:0] response_crc = 16'hffff;
    reg [RESPONSE_POSITION_WIDTH-1:0] response_position = 0;
    reg [RESPONSE_LENGTH_WIDTH-1:0] response_bytes = 18;
    wire [READBACK_INDEX_WIDTH-1:0] readback_bit_position =
        (response_position - 16 + (request_opcode == 9 ? STATE_BYTES : 0)) << 3;
    wire [3:0] header_byte_position = response_position[3:0] - 4'd2;
    reg snapshot_hold = 0;
    reg set_input = 0;
    wire engine_busy;
    wire [READBACK_BYTES * 8 - 1:0] readback;
    reg autonomous = 0;
    reg [63:0] tick_limit = 0;
    wire autonomous_allowed = phase <= VALIDATE || (phase >= TRANSMIT && !snapshot_hold);
    wire run = autonomous && autonomous_allowed;

    assign busy = engine_busy || phase >= VALIDATE;

    assign receive_accept = phase <= RECEIVE;
    compiled_circuit circuit (
        .clock(clock), .set_input(set_input),
        .input_index(request_argument[31:0]), .input_value(request_argument[32]),
        .run(run), .tick_limit(tick_limit), .busy(engine_busy), .tick_count(tick_count),
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

    reg [7:0] response_byte;
    always @* begin
        if (response_position == 0)
            response_byte = 8'h5a;
        else if (response_position == 1)
            response_byte = 8'ha5;
        else if (response_position < 16)
            response_byte = response_header[{header_byte_position, 3'b0} +: 8];
        else if (response_position < response_bytes - 2)
            response_byte = readback[readback_bit_position +: 8];
        else if (response_position == response_bytes - 2)
            response_byte = response_crc[7:0];
        else
            response_byte = response_crc[15:8];
    end

    always @(posedge clock) begin
        set_input <= 0;
        transmit_start <= 0;
        if (receive_error)
            fault <= 1;
        if (receive_error && phase <= RECEIVE) begin
            phase <= FIND_FIRST;
        end else begin
            case (phase)
                FIND_FIRST: if (receive_ready && receive_data == 8'ha5)
                    phase <= FIND_SECOND;
                FIND_SECOND: if (receive_ready) begin
                    if (receive_data == 8'h5a) begin
                        phase <= RECEIVE;
                        request_position <= 0;
                        request_crc <= 16'hffff;
                    end else if (receive_data != 8'ha5) begin
                        phase <= FIND_FIRST;
                    end
                end
                RECEIVE: if (receive_ready) begin
                    if (request_position < 12) begin
                        request[request_position * 8 +: 8] <= receive_data;
                        request_crc <= crc16(request_crc, receive_data);
                    end else if (request_position == 12) begin
                        received_crc[7:0] <= receive_data;
                    end else begin
                        received_crc[15:8] <= receive_data;
                        phase <= VALIDATE;
                    end
                    request_position <= request_position + 1;
                end
                VALIDATE: begin
                    if (request_crc != received_crc || request[95:80] != 1) begin
                        fault <= 1;
                        phase <= FIND_FIRST;
                    end else begin
                        phase <= BOUNDARY;
                    end
                end
                BOUNDARY: if (!engine_busy)
                    phase <= DISPATCH;
                DISPATCH: begin
                    response_opcode <= request_opcode;
                    snapshot_hold <= !compact_response;
                    phase <= CAPTURE;
                    case (request_opcode)
                        0: if (STATE_BYTES != 0) begin
                            autonomous <= 0;
                        end else begin
                            response_opcode <= 8'hff;
                            fault <= 1;
                        end
                        4: autonomous <= 0;
                        2: if (request_argument[31:0] < RESPONSE_NODE_COUNT && request_argument[63:33] == 0) begin
                            autonomous <= 0;
                            set_input <= 1;
                            phase <= WAIT_COMPLETE;
                        end else begin
                            response_opcode <= 8'hff;
                            fault <= 1;
                        end
                        7, 9: begin end
                        8: if (request_argument >= tick_count) begin
                            autonomous <= 1;
                            tick_limit <= request_argument;
                        end else begin
                            response_opcode <= 8'hff;
                            fault <= 1;
                        end
                        default: begin
                            response_opcode <= 8'hff;
                            fault <= 1;
                        end
                    endcase
                end
                WAIT_COMPLETE: phase <= CAPTURE;
                CAPTURE: begin
                    response_header <= {request_opcode == 9 ? RESPONSE_OUTPUT_COUNT : RESPONSE_NODE_COUNT,
                        request_opcode == 4 ? CIRCUIT_ID : tick_count,
                        response_opcode, request_sequence};
                    response_bytes <= compact_response ? 64'd18 :
                        64'd18 + (request_opcode == 9 ? OUTPUT_BYTES : STATE_BYTES);
                    response_position <= 0;
                    response_crc <= 16'hffff;
                    phase <= TRANSMIT;
                end
                TRANSMIT: begin
                    transmit_data <= response_byte;
                    transmit_start <= 1;
                    if (response_position >= 2 && response_position < response_bytes - 2)
                        response_crc <= crc16(response_crc, response_byte);
                    phase <= TRANSMIT_ACCEPT;
                end
                TRANSMIT_ACCEPT: if (transmit_busy)
                    phase <= TRANSMIT_COMPLETE;
                TRANSMIT_COMPLETE: if (!transmit_busy) begin
                    if (response_position == response_bytes - 1) begin
                        phase <= FIND_FIRST;
                    end else begin
                        response_position <= response_position + 1;
                        phase <= TRANSMIT;
                    end
                end
                default: begin
                    fault <= 1;
                    phase <= FIND_FIRST;
                end
            endcase
        end
    end
endmodule
