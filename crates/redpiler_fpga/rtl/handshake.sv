// Carries a request from the board clock to the circuit clock and its acknowledgement back.
//
// The request and its acknowledgement each cross as a level through two flip-flops.
// What the request carries crosses beside them as it is,
// so it must hold still while `request` or `acknowledged` is high.
module handshake (
    input logic board_clock,
    input logic request,
    output logic acknowledged,
    input logic circuit_clock,
    input logic idle,
    output logic hold,
    output logic apply,
    output logic let_go
);
    logic [1:0] request_crossing = 0;
    logic applied = 0;
    logic acknowledge = 0;
    logic [1:0] acknowledge_crossing = 0;

    assign hold = request_crossing[1];
    assign apply = hold && idle && !applied;
    assign let_go = !hold && applied;
    assign acknowledged = acknowledge_crossing[1];

    always_ff @(posedge circuit_clock) begin
        request_crossing <= {request_crossing[0], request};
        applied <= hold && (applied || idle);
        acknowledge <= hold && applied && idle;
    end

    always_ff @(posedge board_clock) acknowledge_crossing <= {acknowledge_crossing[0], acknowledge};
endmodule
