`default_nettype none

// Weight FIFO
// 预取和缓冲权重
module WeightFIFO #(
    parameter TILE  = 4,
    parameter WIDTH = 8,
    parameter DEPTH = 8 * 8
) (
    input logic                 clk,
    input logic                 rst,
    // write
    input logic                 wr_en,
    input logic [WIDTH-1:0]     wr_data [0:DEPTH-1],
    output logic                full,
    // read
    input logic                 rd_en,
    output logic [WIDTH-1:0]    rd_data [0:DEPTH-1],
    output logic                empty

);
    reg [WIDTH-1:0] fifo [0:TILE-1][0:DEPTH-1];

    integer i, j;
    always_ff @( posedge clk ) begin
        if (rst) begin
            for (i = 0; i < TILE; i=i+1) begin
                for (j = 0; j < DEPTH; j=j+1) begin
                    fifo[i][j] <= {WIDTH{1'b0}};
                end
            end
        end else begin

        end
    end
endmodule