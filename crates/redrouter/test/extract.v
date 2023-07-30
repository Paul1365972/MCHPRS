module mux_stacked (A, B, S, Y);
  parameter WIDTH = 1;
  
  input [WIDTH-1:0] A, B;
  input S;
  output [WIDTH-1:0] Y;

  generate
    //if (WIDTH < `MIN_WIDTH) begin
    //    wire _TECHMAP_FAIL_ = 1;
    //end
  endgenerate

  assign Y = S ? B : A;
endmodule
