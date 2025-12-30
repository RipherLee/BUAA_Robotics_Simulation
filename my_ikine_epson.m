function q = my_ikine_epson(T, bot)
    % IKINE_EPSON_MDH_ROBUST 健壮版逆运动学求解
    % 输入:
    %   T: 4x4 目标齐次变换矩阵
    %   bot: 你的 SerialLink 机器人对象 (用于辅助计算 T03)
    % 输出:
    %   q: 1x6 关节角
    
    % 1. 提取参数
    d1 = 0.12202+0.198;
    a1 = 0.1;
    a2 = 0.25;
    d4 = 0.25;      % L3 长度
    d6 = 0.0595;    % 法兰长度
    
    % 2. 求解手腕中心 Pc
    Px = T(1,4); Py = T(2,4); Pz = T(3,4);
    ax = T(1,3); ay = T(2,3); az = T(3,3);
    
    xc = Px - d6 * ax;
    yc = Py - d6 * ay;
    zc = Pz - d6 * az;
    
    % 3. 求解 q1
    q1 = atan2(yc, xc);
    
    % 4. 求解 q2, q3 (位置解)
    % 投影到机械臂平面
    r = sqrt(xc^2 + yc^2) - a1; % 水平距离
    s = zc - d1;                % 垂直高度
    
    % 坐标映射：假设 alpha1 = -90, q2 增加对应手臂向上抬起
    % u: 水平延伸方向
    % v: 垂直向上方向
    u = r; 
    v = s; 
    
    D_sq = u^2 + v^2;
    L2 = a2;
    L3 = d4;
    
    % 余弦定理
    % 这里的t3比定义的theta3大90度
    % 39°<=t3<=315°
    sin_t3 = (D_sq - L2^2 - L3^2) / (2 * L2 * L3);
    
    % --- 关键：工作空间检查 ---
    if sin_t3 > 1
        warning('目标点超出工作空间 (过远)，将伸直手臂计算。Diff: %.4f', sqrt(D_sq) - (L2+L3));
        sin_t3 = 1; 
    elseif sin_t3 < -1
        warning('目标点超出工作空间 (过近)，将折叠手臂计算。');
        sin_t3 = -1;
    end
    
    % 构型选择：Epson C3 默认肘部朝上/外凸，通常 sin_t3 > 0 或 < 0 取决于坐标定义
    % 这里取负值尝试（根据之前的反馈）
    cos_t3 = sqrt(1 - sin_t3^2); 
    
    theta3_geo = -atan2(sin_t3, cos_t3);
    
    % 求解 theta2
    beta = atan2(v, u);
    psi  = atan2(L3 * sin(theta3_geo+pi/2), L2 + L3 * cos(theta3_geo+pi/2));
    theta2_geo = pi/2- beta - psi;
    
    % 映射回 MDH 关节角
    q3 = theta3_geo; 
    q2 = theta2_geo;
    
    % 5. 求解 q4, q5, q6 (姿态解 - 鲁棒方法)
    % 利用工具箱计算前三轴的精确变换矩阵 T03
    % 这样避免了手写 R03 公式出错
    
    % 计算 T03 (仅利用 q1, q2, q3)
    % 这里的 link 1-3 变换
    T01 = bot.links(1).A(q1);
    T12 = bot.links(2).A(q2);
    T23 = bot.links(3).A(q3);
    T03 = T01 * T12 * T23;
    
    % 提取旋转部分
    R03 = T03.R; % 如果是 RTB 10.x 使用 .R，旧版用 T03(1:3, 1:3)
    
    % 计算手腕姿态 R36
    R_target = T(1:3, 1:3);
    R36 = R03' * R_target;
    
    % 解析 Z-Y-Z 欧拉角 (对应 Link 4,5,6 的 alpha: 90, -90, 90)
    % 这里的解析公式依赖于 Link 4,5,6 的具体 alpha 设置
    % 对于 standard Epson:
    r13 = R36(1,3); r23 = R36(2,3); r33 = R36(3,3);
    r31 = R36(3,1); r32 = R36(3,2);
    
    q5 = atan2(sqrt(r13^2 + r23^2), r33);
    
    if abs(sin(q5)) > 1e-5
        q4 = atan2(r23, r13);
        q6 = atan2(r32, -r31);
    else
        q4 = 0;
        q6 = atan2(-R36(1,2), R36(1,1));
    end
    
    % 6. 组合与归一化
    q = [q1, q2, q3, q4, q5, q6];
    
    % 归一化 q1, q4, q5, q6 到 [-pi, pi]
    % q2, q3 保持原值以维持构型
    q(1) = atan2(sin(q(1)), cos(q(1)));
    q(4:6) = atan2(sin(q(4:6)), cos(q(4:6)));
end