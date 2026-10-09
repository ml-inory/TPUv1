`default_nettype none

// 将WeightFIFO数据整形成ROW * COL
module WeightReshape #(
    parameter WIDTH     = 8,
    parameter ROW       = 8,
    parameter COL       = 8
) (
    input logic  [WIDTH*ROW*COL-1:0]    din,
    output logic [WIDTH-1:0]            dout [0:ROW-1][0:COL-1]
);
    integer i, j;

    always_comb begin
        for (i = 0; i < ROW; i=i+1) begin
            for (j = 0; j < COL; j=j+1) begin
                // indexed part select: [base +: WIDTH] keeps the base a variable,
                // the plain [msb:lsb] form would need constant bounds
                dout[i][j] = din[(i * COL + j) * WIDTH +: WIDTH];
            end
        end
    end
    
endmodule

`default_nettype wire
