`default_nettype none

// Weight FIFO
// 预取和缓冲权重
module WeightFIFO #(
    parameter TILE  = 4,
    parameter WIDTH = 8,
    parameter DEPTH = 8 * 8
) (
    input logic                     clk,
    input logic                     rst,
    // write
    input logic                     wr_en,
    input logic [WIDTH*DEPTH-1:0]   wr_data,
    output logic                    full,
    // read
    input logic                     rd_en,
    output logic [WIDTH*DEPTH-1:0]  rd_data,
    output logic                    empty
);
    localparam LINE_SIZE = WIDTH * DEPTH;
    reg [LINE_SIZE-1:0] fifo [0:TILE-1];
    reg [7:0] fifo_size;

    assign full = (fifo_size == TILE);
    assign empty = (fifo_size == 8'b0);

    integer i;
    always_ff @( posedge clk ) begin
        if (rst) begin
            for (i = 0; i < TILE; i=i+1) begin
                fifo[i] <= {LINE_SIZE{1'b0}};
            end
            fifo_size <= 8'b0;
        end else begin
            case ({wr_en & ~full, rd_en & ~empty})
                2'b10: begin
                    fifo[fifo_size] <= wr_data;
                    fifo_size <= fifo_size + 1'b1;
                end
                2'b01: begin
                    rd_data <= fifo[0];
                    for (i = 0; i < fifo_size-1; i=i+1) fifo[i] <= fifo[i+1];
                    fifo_size <= fifo_size - 1'b1;
                end
                2'b11: begin
                    rd_data <= fifo[0];
                    for (i = 0; i < fifo_size-1; i=i+1) fifo[i] <= fifo[i+1];
                    fifo[fifo_size-1] <= wr_data;   // 移位腾出的槽位
                end
            endcase
        end
    end
endmodule