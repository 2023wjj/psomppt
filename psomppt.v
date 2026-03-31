module pso_mppt_top (
    input  wire        clk,        // 系统时钟
    input  wire        rst_n,      // 异步复位
    input  wire [7:0]  v_in,       // 8位电压输入
    input  wire [7:0]  i_in,       // 8位电流输入
    output reg         pwm_out     // PWM输出
);

    // --- 参数定义 ---
    parameter PARTICLE_NUM = 3;    // 粒子数量
    parameter PWM_PERIOD   = 255;  // PWM周期 (8位分辨率)
    
    // PSO 权重参数 (定点数简化：w=0.5, c1=1, c2=1)
    // 速度更新公式: v = w*v + c1*r1*(pbest-x) + c2*r2*(gbest-x)
    
    // --- 内部寄存器 ---
    reg [1:0]  state; //状态机
    reg [7:0]  timer; //pwm方波产生计时器
    reg [7:0]  sample_cnt;
    
    reg [15:0] power_curr;  //当前功率
    reg [7:0]  pos [0:PARTICLE_NUM-1];   // 粒子当前位置 (占空比)
    reg [7:0]  v_abs [0:PARTICLE_NUM-1]; // 粒子速度
    reg [15:0] pbest_val [0:PARTICLE_NUM-1];
    reg [7:0]  pbest_pos [0:PARTICLE_NUM-1];
    reg [15:0] gbest_val;
    reg [7:0]  gbest_pos;
    
    integer i;

    // --- 状态机定义 ---
    localparam IDLE   = 2'd0,
               SAMPLE = 2'd1, // 等待电容稳定并采样
               UPDATE = 2'd2; // 更新PSO参数

    // --- 1. 核心PSO逻辑 ---g
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            sample_cnt <= 0;
            gbest_val <= 0;
            gbest_pos <= 8'h7F; // 初始占空比50%
            for (i=0; i<PARTICLE_NUM; i=i+1) bein
                pos[i] <= (i + 1) * 60; // 均匀分布初始位置
                pbest_val[i] <= 0;
                v_abs[i] <= 0;
            end
        end else begin
            case (state)
                IDLE: begin
                    sample_cnt <= 0;
                    state <= SAMPLE;
                end

                SAMPLE: begin
                    // 模拟采样等待：给硬件电路时间响应占空比变化
                    if (sample_cnt < 8'd200) 
                        sample_cnt <= sample_cnt + 1;
                    else begin
                        sample_cnt <= 0;
                        power_curr <= v_in * i_in; // 获取功率
                        state <= UPDATE;
                    end
                end

                UPDATE: begin
                    // 更新当前粒子的 pbest 和 gbest
                    if (power_curr > pbest_val[i]) begin
                        pbest_val[i] <= power_curr;
                        pbest_pos[i] <= pos[i];
                    end
                    
                    if (power_curr > gbest_val) begin
                        gbest_val <= power_curr;
                        gbest_pos <= pos[i];
                    end

                    // 简化的速度与位置更新 (硬件友好型)
                    // 实际应用中需加入随机数 r1, r2
                    pos[i] <= (pos[i] + (gbest_pos >> 1) - (pos[i] >> 1)); 
                    
                    i <= (i == PARTICLE_NUM - 1) ? 0 : i + 1;
                    state <= SAMPLE;
                end
            endcase
        end
    end

    // --- 2. PWM 产生电路 ---
    // 使用当前最优位置 gbest_pos 作为占空比控制
    reg [7:0] pwm_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_cnt <= 0;
            pwm_out <= 0;
        end else begin
            pwm_cnt <= pwm_cnt + 1;
            pwm_out <= (pwm_cnt < gbest_pos) ? 1'b1 : 1'b0;
        end
    end

endmodule