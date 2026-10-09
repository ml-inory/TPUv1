`default_nettype none

// MXU 计算控制状态机
// IDLE -> LOAD_WEIGHT -> COMPUTE -> DRAIN -> DONE -> IDLE
//
// 时序约定（与 rtl/weight_fifo.sv、rtl/mxu 对齐）：
//   * WeightFIFO 的 rd_data 是寄存器输出：weight_rd_en 弹出的那一片权重要到
//     下一个时钟才出现在 rd_data 上，所以 LOAD_WEIGHT 分两步：
//       第 1 拍  weight_rd_en=1        把权重从 FIFO 弹出来
//       第 2 拍  load_weight=1         此时 rd_data 才是刚弹出的那片权重
//   * psum_out_valid 是 MXU 底部一行各列 valid 的“与”：实测（ROW=4, COL=3）
//     在 act 流开始后 ROW-1 拍拉高、最后一个 act 后 ROW-1 拍落下。所以 DRAIN
//     先等它拉高（说明结果开始流出），再等它落下（说明结果流完了）。
//   * 输出始终 Moore 型：每个状态都给全部输出赋值（先给默认值，避免锁存，
//     也避免某个输出在上一个状态被拉高后一直保持）。
module MXU_Controller (
    input logic  clk,
    input logic  rst,
    // 计算开始
    input logic  start,
    // 权重控制
    output logic load_weight,
    output logic weight_rd_en,
    input  logic weight_fifo_empty,
    // 激活控制
    output logic act_rd_en,
    output logic act_in_valid,
    input  logic [15:0] act_cycles,
    // 部分和控制
    input  logic psum_out_valid,
    // 完成握手：DONE 状态拉高一拍
    output logic done
);
    // 状态类型定义
    typedef enum logic [2:0] {
        IDLE        = 3'b000,
        LOAD_WEIGHT = 3'b001,
        COMPUTE     = 3'b010,
        DRAIN       = 3'b011,
        DONE        = 3'b100
    } state_t;

    // 当前状态和下一状态
    state_t cur_state, next_state;

    // 状态信号
    logic weight_load_done;
    logic compute_done;
    logic drain_done;

    // 握手标志
    logic weight_data_ready;   // FIFO 弹出的权重已经出现在 rd_data 上
    logic drain_seen_valid;    // DRAIN 里已经见到 psum_out_valid 拉高

    // 状态寄存器
    always_ff @(posedge clk) begin
        if (rst)
            cur_state <= IDLE;
        else
            cur_state <= next_state;
    end

    // 下一状态组合逻辑
    always_comb begin
        next_state = cur_state;  // 默认保持
        case (cur_state)
            IDLE: begin
                if (start)
                    next_state = LOAD_WEIGHT;
            end

            LOAD_WEIGHT: begin
                if (weight_load_done)
                    next_state = COMPUTE;
            end

            COMPUTE: begin
                if (compute_done)
                    next_state = DRAIN;
            end

            DRAIN: begin
                if (drain_done)
                    next_state = DONE;
            end

            DONE: begin
                next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    // 激活计数：COMPUTE 里每拍（act_in_valid=1）累加，离开 COMPUTE 清零
    logic [15:0] act_cnt;
    logic act_done;

    always_ff @(posedge clk) begin
        if (rst)
            act_cnt <= 16'b0;
        else if (cur_state == COMPUTE && act_in_valid)
            act_cnt <= act_cnt + 1'b1;
        else if (cur_state != COMPUTE)
            act_cnt <= 16'b0;
    end

    // act_cycles = 0 视作 1 拍，避免 (act_cycles-1) 下溢后永远算不完
    assign act_done = (act_cycles == 16'd0) || (act_cnt == act_cycles - 1'b1);

    // 权重装载 / 排空握手标志
    always_ff @(posedge clk) begin
        if (rst) begin
            weight_data_ready <= 1'b0;
            drain_seen_valid  <= 1'b0;
        end else begin
            // LOAD_WEIGHT：FIFO 非空的那一拍弹出，下一拍数据有效
            if (cur_state != LOAD_WEIGHT)
                weight_data_ready <= 1'b0;
            else if (!weight_data_ready && !weight_fifo_empty)
                weight_data_ready <= 1'b1;

            // DRAIN：先记录 valid 拉高，再等它落下
            if (cur_state != DRAIN)
                drain_seen_valid <= 1'b0;
            else if (psum_out_valid)
                drain_seen_valid <= 1'b1;
        end
    end

    // 输出逻辑（Moore 型）：先给默认值，再按状态覆盖，任何状态都不会漏赋值
    always_comb begin
        load_weight      = 1'b0;
        weight_rd_en     = 1'b0;
        act_rd_en        = 1'b0;
        act_in_valid     = 1'b0;
        weight_load_done = 1'b0;
        compute_done     = 1'b0;
        drain_done       = 1'b0;
        done             = 1'b0;

        case (cur_state)
            IDLE: begin
                // 全部默认 0，等 start
            end

            LOAD_WEIGHT: begin
                act_rd_en = 1'b0;                     // 不灌激活
                if (!weight_data_ready) begin
                    weight_rd_en = 1'b1;              // 第一步：弹出权重
                end else begin
                    load_weight      = 1'b1;          // 第二步：装载 rd_data 上的权重
                    weight_load_done = 1'b1;          // -> COMPUTE
                end
            end

            COMPUTE: begin
                act_rd_en    = 1'b1;                  // 按节奏读激活
                act_in_valid = 1'b1;
                compute_done = act_done;              // -> DRAIN
            end

            DRAIN: begin
                // 新激活已停，等流水线里的部分和流完
                drain_done = drain_seen_valid && ~psum_out_valid;
            end

            DONE: begin
                done = 1'b1;                          // 完成握手（一拍）
            end

            default: begin
                // 和 IDLE 一样：全部默认 0
            end
        endcase
    end

endmodule

`default_nettype wire
