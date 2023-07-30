module counter(D, LOAD, CLK, RST, Q);
  input LOAD, CLK, RST;
  input [7:0] D;
  output reg [7:0] Q;
  
  always@(posedge CLK) begin
    if(RST)
  		Q <= 0;
    else if(LOAD)
      Q <= D;
    else
      Q <= Q + 1;
  end
endmodule
