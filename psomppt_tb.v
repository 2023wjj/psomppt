module pso_mppt_top (
    input  wire        clk,        // 100MHz
    input  wire        rst_n,      
    input  wire [7:0]  v_in,       // ADC输入 (0.1V/step)
    input  wire [7:0]  i_in,       // ADC输入 (0.1A/step)
    output reg         pwm_out     
);

    // --- 参数与索引 ---
    parameter PARTICLE_NUM = 3;
    reg [2:0]  state;
    reg [7:0]  sample_cnt;
    reg [1:0]  p_idx;              // 粒子索引: 0, 1, 2
    
    // --- PSO 核心寄存器 ---
    reg [15:0] power_curr;
    reg [7:0]  pos [0:PARTICLE_NUM-1];
    reg signed [8:0] vel [0:PARTICLE_NUM-1]; // 有符号速度
    reg [15:0] pbest_val [0:PARTICLE_NUM-1];
    reg [7:0]  pbest_pos [0:PARTICLE_NUM-1];
    reg [15:0] gbest_val;
    reg [7:0]  gbest_pos;

    // --- 状态机 ---
    localparam IDLE   = 3'd0,
               SAMPLE = 3'd1, // 电路响应期
               EVAL   = 3'd2, // 评估并更新最优值
               UPD_V  = 3'd3, // 计算速度
               UPD_P  = 3'd4; // 更新位置+边界检查

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            p_idx <= 0;
            gbest_val <= 0;
            gbest_pos <= 8'h7F;
            pos[0] <= 8'd64;  vel[0] <= 9'd0; pbest_val[0] <= 0;
            pos[1] <= 8'd128; vel[1] <= 9'd0; pbest_val[1] <= 0;
            pos[2] <= 8'd192; vel[2] <= 9'd0; pbest_val[2] <= 0;
        end else begin
            case (state)
                IDLE: state <= SAMPLE;

                SAMPLE: begin
                    // 等待 DCDC 在当前 pos[p_idx] 下稳定
                    if (sample_cnt < 8'd250) 
                        sample_cnt <= sample_cnt + 1;
                    else begin
                        sample_cnt <= 0;
                        power_curr <= v_in * i_in; 
                        state <= EVAL;
                    end
                end

                EVAL: begin
                    if (power_curr > pbest_val[p_idx]) begin
                        pbest_val[p_idx] <= power_curr;
                        pbest_pos[p_idx] <= pos[p_idx];
                    end
                    if (power_curr > gbest_val) begin
                        gbest_val <= power_curr;
                        gbest_pos <= pos[p_idx];
                    end
                    state <= UPD_V;
                end

                UPD_V: begin
                    // v = 0.5*v + 0.25*(pb-x) + 0.25*(gb-x)
                    vel[p_idx] <= (vel[p_idx] >>> 1) + 
                                  (($signed({1'b0, pbest_pos[p_idx]}) - $signed({1'b0, pos[p_idx]})) >>> 2) + 
                                  (($signed({1'b0, gbest_pos})        - $signed({1'b0, pos[p_idx]})) >>> 2);
                    state <= UPD_P;
                end

                UPD_P: begin
                    // 位置更新与饱和截断
                    if ($signed({1'b0, pos[p_idx]}) + vel[p_idx] > 9'd255)
                        pos[p_idx] <= 8'd255;
                    else if ($signed({1'b0, pos[p_idx]}) + vel[p_idx] < 9'd0)
                        pos[p_idx] <= 8'd0;
                    else
                        pos[p_idx] <= pos[p_idx] + vel[p_idx][7:0];

                    p_idx <= (p_idx == PARTICLE_NUM - 1) ? 2'd0 : p_idx + 2'd1;
                    state <= SAMPLE;
                end
            endcase
        end
    end

    // --- PWM 产生 (核心修正: 跟随当前评估的粒子) ---
    reg [7:0] pwm_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_cnt <= 0;
            pwm_out <= 0;
        end else begin
            pwm_cnt <= pwm_cnt + 1;
            // 此时物理电路会根据正在评估的粒子 pos[p_idx] 产生功率反馈
            pwm_out <= (pwm_cnt < pos[p_idx]) ? 1'b1 : 1'b0;
        end
    end
endmodule