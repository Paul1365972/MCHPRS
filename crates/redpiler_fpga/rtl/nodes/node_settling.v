module node_settling #(
    parameter STALE = 0
) (
    input wire clock,
    input wire changed,
    output wire update
);
    generate
        if (STALE) begin : stale
            reg waiting = 1'b1;
            assign update = !waiting || changed;
            always @(posedge clock)
                waiting <= waiting && !changed;
        end else begin : settled
            assign update = 1'b1;
        end
    endgenerate
endmodule
