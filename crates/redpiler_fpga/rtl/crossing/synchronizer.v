// Brings a level from another clock domain, or from a pin, into clock's domain.
module synchronizer (
    input wire clock,
    input wire signal,
    output wire synchronized
);
    (* ASYNC_REG = "TRUE" *) reg first = 0;
    (* ASYNC_REG = "TRUE" *) reg second = 0;

    assign synchronized = second;

    always @(posedge clock) begin
        first <= signal;
        second <= first;
    end
endmodule
