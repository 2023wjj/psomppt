module pso_mppt_top (
    input  wire        clk,        // 系统时钟
    input  wire        rst_n,      // 异步复位
    input  wire [7:0]  v_in,       // 8位电压输入
    input  wire [7:0]  i_in,       // 8位电流输入
    output reg         pwm_out     // PWM输出
);

    // --- 参数定义 ---
    parameter PARTICLE_NUM = 3;    
    
    // --- 内部寄存器 (ASIC 优化位宽) ---
    reg [2:0]  state;              // 状态机状态
    reg [7:0]  sample_cnt;         // 采样等待计数
    reg [1:0]  p_idx;              // 当前正在处理的粒子索引 
    
    reg [15:0] power_curr;         // 当前功率 P = V * I
    reg [7:0]  pos [0:PARTICLE_NUM-1];   // 粒子位置 (0-255 占空比)
    
    // 速度项，有符号数，9位位宽可覆盖 -255 到 255 范围
    reg signed [8:0] vel [0:PARTICLE_NUM-1]; 

    reg [15:0] pbest_val [0:PARTICLE_NUM-1];
    reg [7:0]  pbest_pos [0:PARTICLE_NUM-1];
    reg [15:0] gbest_val;
    reg [7:0]  gbest_pos;

    // --- 状态机定义 ---
    localparam IDLE   = 3'd0,
               SAMPLE = 3'd1, // 等待硬件稳定
               EVAL   = 3'd2, // 更新 pbest 和 gbest
               UPD_V  = 3'd3, // 更新速度
               UPD_P  = 3'd4; // 更新位置并进行边界检查

    // --- 1. 核心 PSO 逻辑 (带速度项) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            sample_cnt <= 0;
            p_idx <= 0;
            gbest_val <= 0;
            gbest_pos <= 8'h7F;
            // 粒子初始化
            pos[0] <= 8'd60;  vel[0] <= 9'd0; pbest_val[0] <= 0;
            pos[1] <= 8'd120; vel[1] <= 9'd0; pbest_val[1] <= 0;
            pos[2] <= 8'd180; vel[2] <= 9'd0; pbest_val[2] <= 0;
        end else begin
            case (state)
                IDLE: begin
                    state <= SAMPLE;
                end

                SAMPLE: begin
                    if (sample_cnt < 8'd200) 
                        sample_cnt <= sample_cnt + 1;
                    else begin
                        sample_cnt <= 0;
                        power_curr <= v_in * i_in; 
                        state <= EVAL;
                    end
                end

                EVAL: begin
                    // 更新个体最优 pbest
                    if (power_curr > pbest_val[p_idx]) begin
                        pbest_val[p_idx] <= power_curr;
                        pbest_pos[p_idx] <= pos[p_idx];
                    end
                    // 更新全局最优 gbest
                    if (power_curr > gbest_val) begin
                        gbest_val <= power_curr;
                        gbest_pos <= pos[p_idx];
                    end
                    state <= UPD_V;
                end

                UPD_V: begin
                    // 标准公式: v = w*v + c1*(pbest-x) + c2*(gbest-x)
                    // 硬件简化方案: w=0.5 (>>>1), c1=0.25 (>>>2), c2=0.25 (>>>2)
                    // $signed 是为了确保减法按有符号数处理
                    vel[p_idx] <= (vel[p_idx] >>> 1) + 
                                  (($signed({1'b0, pbest_pos[p_idx]}) - $signed({1'b0, pos[p_idx]})) >>> 2) + 
                                  (($signed({1'b0, gbest_pos})        - $signed({1'b0, pos[p_idx]})) >>> 2);
                    state <= UPD_P;
                end

                UPD_P: begin
                    // 位置更新: x = x + v
                    // 需要进行饱和截断 (Clipping)，防止占空比超出 0-255 导致回绕
                    if ($signed({1'b0, pos[p_idx]}) + vel[p_idx] > 9'd255)
                        pos[p_idx] <= 8'd255;
                    else if ($signed({1'b0, pos[p_idx]}) + vel[p_idx] < 9'd0)
                        pos[p_idx] <= 8'd0;
                    else
                        pos[p_idx] <= pos[p_idx] + vel[p_idx][7:0];

                    // 切换到下一个粒子
                    p_idx <= (p_idx == PARTICLE_NUM - 1) ? 2'd0 : p_idx + 2'd1;
                    state <= SAMPLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

    // --- 2. PWM 产生电路 (使用 gbest 控制) ---
    reg [7:0] pwm_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_cnt <= 0;
            pwm_out <= 0;
        end else begin
            pwm_cnt <= pwm_cnt + 8'd1;
            pwm_out <= (pwm_cnt < gbest_pos) ? 1'b1 : 1'b0;
        end
    end

endmodule