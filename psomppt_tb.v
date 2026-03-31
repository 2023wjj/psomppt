`timescale 1ns / 1ps

module tb_pso_mppt_realistic_pv();
    reg clk;
    reg rst_n;
    wire [7:0] v_in, i_in;
    wire pwm_out;

    // --- 物理环境参数 ---
    real G = 1000.0;      // 光照强度 (W/m^2)
    real T = 25.0;        // 温度 (摄氏度)
    
    // 标准测试条件 (STC) 下的 PV 参数
    parameter real V_OC_STC = 21.0;
    parameter real I_SC_STC = 5.2;
    parameter real V_MPP_STC = 17.5;
    parameter real I_MPP_STC = 4.8;
    parameter real R_LOAD = 10.0; // 外部负载电阻

    // 内部计算变量
    real duty_cycle = 0.0;
    real v_pv_phys = 18.0;
    real i_pv_phys = 0.0;
    real c1, c2;

    // --- 实例化 PSO 模块 ---
    pso_mppt_top uut (
        .clk(clk), .rst_n(rst_n), .v_in(v_in), .i_in(i_in), .pwm_out(pwm_out)
    );

    // --- 初始化 PV 曲线常数 ---
    initial begin
        // 根据 STC 参数预计算曲线形态
        c2 = (V_MPP_STC / V_OC_STC - 1.0) / $ln(1.0 - I_MPP_STC / I_SC_STC);
        c1 = (1.0 - I_MPP_STC / I_SC_STC) * $exp(-V_MPP_STC / (c2 * V_OC_STC));
    end

    // --- 时钟与 PWM 采样 ---
    initial clk = 0;
    always #5 clk = ~clk;

    real pwm_sum = 0;
    integer window = 0;
    always @(posedge clk) begin
        pwm_sum = pwm_sum + (pwm_out ? 1.0 : 0.0);
        window = window + 1;
        if (window >= 256) begin
            duty_cycle = pwm_sum / 256.0;
            pwm_sum = 0; window = 0;
        end
    end

    // --- 核心物理模型迭代 ---
    // 模拟电阻负载下的工作点移动
    always @(posedge clk) begin
        real D = (duty_cycle > 0.95) ? 0.95 : duty_cycle;
        // 等效电阻 Req = R_load * (1-D)^2
        real r_eq = R_LOAD * (1.0 - D) * (1.0 - D);
        
        // 使用 Newton-Raphson 或简化迭代寻找 I_pv 和 V_pv 的交点
        // 这里使用简化指数模型计算物理反馈
        // I = G_ratio * I_sc * [1 - c1*(exp(V/(c2*V_oc)) - 1)]
        // 同时 V = I * r_eq
        // 简单迭代模拟电容充放电平衡
        v_pv_phys <= v_pv_phys + ( (i_pv_phys * r_eq) - v_pv_phys ) * 0.01;
        
        // PV 特性方程更新电流
        if (v_pv_phys < V_OC_STC)
            i_pv_phys = (G/1000.0) * I_SC_STC * (1.0 - c1 * ($exp(v_pv_phys/(c2 * V_OC_STC)) - 1.0));
        else
            i_pv_phys = 0.0;
            
        if (i_pv_phys < 0) i_pv_phys = 0;
    end

    // --- 信号量化给 ASIC (0.1V/0.1A 分辨率) ---
    assign v_in = $rtoi(v_pv_phys * 10.0);
    assign i_in = $rtoi(i_pv_phys * 10.0);

    // --- 仿真控制与监控 ---
    initial begin
        rst_n = 0; #100; rst_n = 1;
        
        $display("Realistic PV Simulation Start...");
        $monitor("T:%t | D:%f | Vpv:%f | Ipv:%f | P:%f", 
                 $time, duty_cycle, v_pv_phys, i_pv_phys, v_pv_phys * i_pv_phys);

        // 模拟光照突变测试 PSO 动态响应
        #500000;
        G = 600.0; // 光照减弱
        $display(">>> 光照突降至 600W/m^2");
        
        #500000;
        $finish;
    end
endmodule