`default_nettype none

// Unified Buffer
module UB #(
    parameter WIDTH = 8,
    parameter DEPTH = 256
) (
    input logic                         clk,
    input logic                         rst,
    // Read
    input logic                         rd_en,
    input logic [$clog2(DEPTH)-1:0]     rd_addr,
    output logic [WIDTH-1:0]            rd_data,
    // Write
    input logic                         wr_en,
    input logic [$clog2(DEPTH)-1:0]     wr_addr,
    input logic [WIDTH-1:0]             wr_data
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < DEPTH; i=i+1) begin
                mem[i] <= {WIDTH{1'b0}};
            end
            rd_data <= {WIDTH{1'b0}};
        end else begin
            if (rd_en) begin
                rd_data <= mem[rd_addr];
            end

            if (wr_en) begin
                mem[wr_addr] <= wr_data;
            end
        end
    end
endmodule