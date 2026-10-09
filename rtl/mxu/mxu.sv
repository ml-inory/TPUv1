`default_nettype none

// systolic array

module MXU #(
    parameter   ROW   = 8,
    parameter   COL   = 8,
    parameter   ACT_WIDTH       = 8,
    parameter   WEIGHT_WIDTH    = 8,
    parameter   ACC_WIDTH       = 32

) (
    input logic clk,
    input logic rst,
    // weight
    input logic load_weight,
    input logic [WEIGHT_WIDTH-1:0] weight_in [0:ROW-1][0:COL-1],
    // activation/input feature, per row
    input logic act_in_valid,
    input logic [ACT_WIDTH-1:0] act_in [0:ROW-1],
    // partial sum in, per column
    input logic psum_in_valid, 
    input logic [ACC_WIDTH-1:0] psum_in [0:COL-1],
    // partial sum out
    output logic psum_out_valid,
    output logic [ACC_WIDTH-1:0] psum_out [0:COL-1]
);
    logic act_in_valid_wire [0:ROW-1][0:COL];
    logic [ACT_WIDTH-1:0] act_in_wire [0:ROW-1][0:COL];
    logic psum_in_valid_wire [0:ROW-1][0:COL-1];
    logic [ACC_WIDTH-1:0] psum_wire [0:ROW][0:COL-1];
    logic psum_out_valid_wire [0:ROW-1][0:COL-1];

    // psum_out_valid：与对齐后的 psum_out 同一拍，表示这一拍上 COL 个 psum
    // 都是同一个 wavefront 的结果（最后一列底部的 PE 算完即全部算完）
    assign psum_out_valid = psum_out_valid_wire[ROW-1][COL-1];

    genvar i, j;
    generate
        for (i = 0; i < ROW; i++) begin : gen_row_input
            // 激活延迟
            delay_chain #(.WIDTH(ACT_WIDTH), .DEPTH(i)) act_delay (
                .clk(clk), .rst(rst),
                .din(act_in[i]), .dout(act_in_wire[i][0])
            );
            // valid 同步延迟
            delay_chain #(.WIDTH(1), .DEPTH(i)) valid_delay (
                .clk(clk), .rst(rst),
                .din(act_in_valid), .dout(act_in_valid_wire[i][0])
            );
        end

        for (j = 0; j < COL; j++) begin: gen_col_input
            // 部分和延迟
            delay_chain #(.WIDTH(ACC_WIDTH), .DEPTH(j)) psum_delay (
                .clk(clk), .rst(rst),
                .din(psum_in[j]), .dout(psum_wire[0][j])
            );
        end

        for (i = 0; i < ROW; i++) begin : gen_row
            for (j = 0; j < COL; j++) begin : gen_col
                PE pe_inst (
                    .clk            (clk),
                    .rst            (rst),
                    // weight   
                    .load_weight    (load_weight),
                    .weight_in      (weight_in[i][j]),
                    // act  
                    .act_in         (act_in_wire[i][j]),
                    .act_in_valid   (act_in_valid_wire[i][j]),   // 每行独立
                    .act_out        (act_in_wire[i][j+1]),
                    .act_out_valid  (act_in_valid_wire[i][j+1]),
                    // psum
                    .psum_in        (psum_wire[i][j]),
                    .psum_out       (psum_wire[i+1][j]),
                    .psum_out_valid (psum_out_valid_wire[i][j])
                );
            end
        end

        for (j = 0; j < COL; j++) begin : gen_psum_out
            // 输出对齐：列 j 延迟 (COL-1-j) 拍，把各列的错拍补平，使同一拍上
            // 得到一个完整 wavefront 的 COL 个列结果
            if (j == COL-1) begin : g_pass
                assign psum_out[j] = psum_wire[ROW][j];
            end else begin : g_delay
                delay_chain #(.WIDTH(ACC_WIDTH), .DEPTH(COL-1-j)) out_delay (
                    .clk(clk), .rst(rst),
                    .din(psum_wire[ROW][j]), .dout(psum_out[j])
                );
            end
        end

    endgenerate
endmodule
