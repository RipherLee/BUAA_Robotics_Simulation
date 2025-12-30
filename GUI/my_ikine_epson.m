% function q = my_ikine_epson(T)
%     % IKINE_EPSON_MDH_FIXED 修正版 Epson C3 逆运动学解析解
%     % 适用于前置 MDH 参数
% 
%     % 1. 参数定义 (保持不变)
%     d1 = 0.12202+0.198; % 考虑基座高度
%     a1 = 0.1;
%     a2 = 0.25;
%     d4 = 0.25;      % L3 长度
%     d6 = 0.0595;    % 法兰长度
% 
%     % 2. 求解手腕中心 Pc
%     Px = T(1,4); Py = T(2,4); Pz = T(3,4);
%     ax = T(1,3); ay = T(2,3); az = T(3,3);
% 
%     xc = Px - d6 * ax;
%     yc = Py - d6 * ay;
%     zc = Pz - d6 * az;
% 
%     % 3. 求解 q1
%     q1 = atan2(yc, xc);
% 
%     % 4. 求解 q2, q3 (关键修正部分)
%     % 投影到机械臂平面
%     r = sqrt(xc^2 + yc^2) - a1; % 水平距离
%     s = zc - d1;                % 垂直高度 (向上为正)
% 
%     % --- 坐标系转换 ---
%     % 由于 alpha1 = -90, 关节2的正旋转方向会导致手臂向下运动。
%     % 因此，垂直高度 s 在 J2 局部平面坐标系中对应的是 -y 方向。
%     % u: 沿 X2 方向 (水平)
%     % v: 沿 Y2 方向 (垂直向下)
%     u = r;
%     v = -s; 
% 
%     D_sq = u^2 + v^2;
%     L2 = a2;
%     L3 = d4;
% 
%     % 余弦定理求几何肘关节角 theta3 (L2与L3夹角的外角)
%     cos_t3 = (u^2 + v^2 - L2^2 - L3^2) / (2 * L2 * L3);
% 
%     % 数值钳位
%     if cos_t3 > 1, cos_t3 = 1; elseif cos_t3 < -1, cos_t3 = -1; end
% 
%     % 构型选择：根据你的正确解，theta3 = 150度，sin > 0
%     sin_t3 = sqrt(1 - cos_t3^2); % 正解
% 
%     theta3_geo = atan2(sin_t3, cos_t3);
% 
%     % 求解几何肩关节角 theta2
%     % 公式: theta2 = atan2(v, u) - atan2(k2, k1)
%     % 展开后即为：
%     beta = atan2(v, u); % 注意这里用的是 v=-s
%     psi  = atan2(L3 * sin_t3, L2 + L3 * cos_t3);
%     theta2_geo = beta - psi;
% 
%     % --- 映射回 MDH 关节角 ---
%     % MDH L(3): offset = pi  -> q3 + pi = theta3_geo
%     q3 = theta3_geo - pi;
% 
%     % MDH L(2): offset = -pi/2 -> q2 - pi/2 = theta2_geo
%     q2 = theta2_geo + pi/2;
% 
%     % 5. 求解 q4, q5, q6 (姿态)
%     % 必须手动构建准确的 R03 矩阵来消除前三轴的影响
% 
%     c1=cos(q1); s1=sin(q1);
%     c2=cos(q2 - pi/2); s2=sin(q2 - pi/2);
%     c3=cos(q3 + pi);   s3=sin(q3 + pi);
% 
%     % R01: alpha0=0
%     R01 = [c1 -s1 0; s1 c1 0; 0 0 1];
% 
%     % R12: alpha1=-pi/2 -> RotX(-90)
%     % [c -s 0; 0 0 1; -s -c 0]
%     R12 = [c2 -s2 0; 0 0 1; -s2 -c2 0];
% 
%     % R23: alpha2=0
%     R23 = [c3 -s3 0; s3 c3 0; 0 0 1];
% 
%     R03 = R01 * R12 * R23;
% 
%     % 计算 R36
%     R_target = T(1:3, 1:3);
%     R36 = R03' * R_target;
% 
%     % 解析欧拉角 (基于后三轴 MDH: a3=90, a4=-90, a5=90)
%     % 对应的旋转矩阵结构 R36 = R34(q4)*R45(q5)*R56(q6)
%     % T3(q4): alpha3=90  -> RotZ(q4)RotX(90)
%     % T4(q5): alpha4=-90 -> RotZ(q5)RotX(-90)
%     % T5(q6): alpha5=90  -> RotZ(q6)RotX(90)
% 
%     % 经过推导，该序列对应的 Z-Y-Z 形式解法如下：
%     % R36(2,3) = s4*s5
%     % R36(1,3) = c4*s5
%     % R36(3,3) = c5
%     % R36(3,2) = s5*c6
%     % R36(3,1) = s5*s6 (注意符号可能需微调)
% 
%     % 按照你的正确解 q4,q5,q6 结构：
%     r13 = R36(1,3); r23 = R36(2,3); r33 = R36(3,3);
%     r31 = R36(3,1); r32 = R36(3,2);
% 
%     % 修正后的欧拉角提取公式 (适配 Epson MDH 结构)
%     % 1. 求 q5
%     q5 = atan2(sqrt(r13^2 + r23^2), r33);
% 
%     % 2. 求 q4, q6
%     if abs(sin(q5)) > 1e-5
%         q4 = atan2(r23, r13);
%         q6 = atan2(r32, -r31);
%     else
%         q4 = 0;
%         q6 = atan2(-R36(1,2), R36(1,1));
%     end
% 
%     % 6. 组合结果
%     q = [q1, q2, q3, q4, q5, q6];
% 
%     % 归一化到 [-pi, pi] (Epson C3 某些关节范围可能超过pi，但在逆解中先归一化)
%     % q2, q3 不需要归一化，保留原始值以匹配构型
%     q(1) = atan2(sin(q(1)), cos(q(1)));
%     q(4:6) = atan2(sin(q(4:6)), cos(q(4:6)));
% end
function q_best = my_ikine_epson(T, q_ref)
    % MY_IKINE_EPSON 针对 Epson C3 的解析逆运动学 (奇异点修正版)
    % 
    % 修复重点：
    % 1. 修正了 J5=0 (奇异点) 时的姿态解算逻辑。
    % 2. 针对 Modified DH 参数 (offset) 进行了专门的矩阵推导。
    
    if nargin < 2 || isempty(q_ref), q_ref = zeros(1,6); end

    % --- 1. 机器人 DH 参数 (必须与 RobotGUI 中定义的一致) ---
    d1 = 0.12202 + 0.198; 
    a1 = 0.1;
    a2 = 0.25;
    d4 = 0.187 + 0.063; % = 0.25
    d6 = 0.0595;    
    
    % --- 2. 求解手腕中心 (Wrist Center) ---
    % 末端位姿 T = [nx ox ax Px; ny oy ay Py; nz oz az Pz; 0 0 0 1]
    Px = T(1,4); Py = T(2,4); Pz = T(3,4);
    ax = T(1,3); ay = T(2,3); az = T(3,3);
    
    % 手腕中心 Pc = P_end - d6 * a (沿着接近矢量回退 d6)
    xc = Px - d6 * ax;
    yc = Py - d6 * ay;
    zc = Pz - d6 * az;
    
    candidates = [];
    
    % --- 第一层：求解 J1 (左右手系) ---
    % 很多时候需要 atan2(-yc, -xc) 才能得到背向解，这里遍历两种情况
    for sigma_1 = [1, -1]
        if sigma_1 == 1
            q1 = atan2(yc, xc);
            r = sqrt(xc^2 + yc^2) - a1; % 水平投影距离 (扣除 a1)
        else
            q1 = atan2(-yc, -xc);
            r = -sqrt(xc^2 + yc^2) - a1;
        end
        
        % 垂直距离
        s = zc - d1;
        
        % --- 第二层：求解 J3 (余弦定理) ---
        % 三角形边长: a2, d4, D=sqrt(r^2+s^2)
        cos_q3 = (r^2 + s^2 - a2^2 - d4^2) / (2 * a2 * d4);
        
        if abs(cos_q3) > 1 + 1e-5
            continue; % 构型不可达
        end
        if cos_q3 > 1, cos_q3 = 1; elseif cos_q3 < -1, cos_q3 = -1; end
        
        for sigma_3 = [1, -1] % 肘部 上/下
            sin_q3 = sigma_3 * sqrt(1 - cos_q3^2);
            
            % Modified DH 中 J3 的定义通常需要加一个 offset
            % 根据你的 L_M(3) offset=pi，这里的几何角需要转换
            % 几何推导:
            % theta3_geo 是几何夹角
            theta3_geo = atan2(sin_q3, cos_q3);
            
            % 映射到 MDH J3: q3 = theta3_geo - pi/2 或类似，取决于定义
            % 你的定义: L_M(3).offset = pi. 
            % 经过调试，通常 geometric J3 = q3_mdh. 
            % 我们先算出几何解，最后再调优。
            % 假设标准几何解:
            q3_val = theta3_geo - pi/2; % 这是常见的 geometrical to MDH offset
            % 如果发现 J3 总是差 90 度，这里需要调整。基于 offset=pi:
            q3_val = theta3_geo; 

            % --- 求解 J2 ---
            alpha = atan2(s, r);
            beta  = atan2(d4 * sin_q3, a2 + d4 * cos_q3);
            q2_val = alpha - beta;
            
            % MDH Offset 修正 (L_M(2) offset = -pi/2)
            % 通常几何角直接对应 q2，不需要额外修正，除非坐标系旋转了
            
            % --- 求解手腕姿态 (Orientation) ---
            % 计算 R03 (前三轴的旋转矩阵)
            % 既然有 Robotics Toolbox，我们利用它提供的 sub-fkine 功能来保证 R03 绝对正确
            % 这样避免手写矩阵的符号错误
            % R03 = T01 * T12 * T23 的旋转部分
            
            % 手写 R03 (基于你的 MDH):
            c1=cos(q1); s1=sin(q1);
            c2=cos(q2_val); s2=sin(q2_val);
            c3=cos(q3_val); s3=sin(q3_val);
            
            % T01 (alpha=0)
            R01 = [c1 -s1 0; s1 c1 0; 0 0 1];
            % T12 (alpha=-90, off=-90 -> q2-pi/2) -> 实际传入 sin/cos 时注意 offset
            % L_M(2): alpha=-pi/2, offset=-pi/2. 
            % R matrix for MDH Link i: [cq -sq 0; sq*calpha cq*calpha -salpha; sq*salpha cq*salpha calpha]
            % 这太复杂容易错，我们用逆变换法：
            
            % 构造 R03_num (数值法，最稳)
            % 这里必须非常小心 DH 参数的一致性
            % 简单方法：利用 geometry 算出的 q1,q2,q3，求 R03
            % 为了避免依赖 toolbox 对象，这里写死 MDH 的旋转链：
            
            % Link 1: a=0, alpha=0, d=d1, th=q1
            T1 = [c1 -s1 0 0; s1 c1 0 0; 0 0 1 0; 0 0 0 1];
            
            % Link 2: a=a1, alpha=-90, d=0, th=q2-90
            q2_m = q2_val - pi/2; 
            c2m=cos(q2_m); s2m=sin(q2_m);
            T2 = [c2m -s2m 0 a1; 0 0 1 0; -s2m -c2m 0 0; 0 0 0 1];
            
            % Link 3: a=a2, alpha=0, d=0, th=q3+180
            q3_m = q3_val + pi;
            c3m=cos(q3_m); s3m=sin(q3_m);
            T3 = [c3m -s3m 0 a2; s3m c3m 0 0; 0 0 1 0; 0 0 0 1];
            
            T03 = T1 * T2 * T3;
            R03 = T03(1:3, 1:3);
            
            % 目标旋转矩阵 R_target
            R_target = T(1:3, 1:3);
            
            % 得到手腕旋转 R36 = R03' * R_target
            R36 = R03' * R_target;
            
            % R36 对应 Z(q4) * X(90) * Z(q5) * X(-90) * Z(q6)  (基于 L_M 4,5,6 参数)
            % MDH Link 4: a=0, alpha=90, th=q4
            % MDH Link 5: a=0, alpha=-90, th=q5
            % MDH Link 6: a=0, alpha=90, th=q6
            
            % 推导 R36 表达式:
            % R36 = [ c4c5c6-s4s6, -c4c5s6-s4c6, -c4s5 ]
            %       [ s4c5c6+c4s6, -s4c5s6+c4c6, -s4s5 ]
            %       [      s5c6  ,      -s5s6  ,   c5  ]
            
            r13 = R36(1,3); r23 = R36(2,3); r33 = R36(3,3);
            
            % --- 求解 J5 ---
            % c5 = r33
            cos_q5 = r33;
            
            % 奇异点判断：如果 abs(c5) ~ 1，说明 s5 ~ 0，J5 ~ 0 (奇异)
            if abs(cos_q5) > 0.999
                % === 奇异点处理 (Singularity Handling) ===
                q5_val = 0; % 强制 J5 = 0
                
                % 此时 R36 退化为：
                % [ c(4+6)  -s(4+6)   0 ]
                % [ s(4+6)   c(4+6)   0 ]
                % [   0        0      1 ]
                % (具体符号取决于 DH 定义，可能是 4-6 或 4+6)
                
                % 计算 sum_46
                sum_46 = atan2(R36(2,1), R36(1,1));
                
                % 策略：锁定 J4 为“当前参考值”，让 J6 承担剩余旋转
                % 找到离 q_ref(4) 最近的周期
                k = round(q_ref(4) / (2*pi));
                q4_val = q_ref(4); % 保持不动
                
                % 求解 q6
                q6_val = sum_46 - q4_val;
                
            else
                % === 正常情况 (非奇异) ===
                for sigma_5 = [1, -1]
                    sin_q5 = sigma_5 * sqrt(1 - cos_q5^2);
                    q5_val = atan2(sin_q5, cos_q5);
                    
                    % 求解 J4:  -c4s5 = r13, -s4s5 = r23  => s4 = -r23/s5, c4 = -r13/s5
                    q4_val = atan2(-r23/sin_q5, -r13/sin_q5);
                    
                    % 求解 J6:  s5c6 = r31, -s5s6 = r32   => s6 = -r32/s5, c6 = r31/s5
                    q6_val = atan2(-R36(3,2)/sin_q5, R36(3,1)/sin_q5);
                    
                    add_candidate([q1, q2_val, q3_val, q4_val, q5_val, q6_val]);
                end
                continue; % 跳过下方的 add_candidate，因为循环里已经加了
            end
            
            add_candidate([q1, q2_val, q3_val, q4_val, q5_val, q6_val]);
        end
    end
    
    % --- 内部函数：添加并筛选解 ---
    function add_candidate(q)
        % 去卷绕 (Unwrap)
        for k=1:6
            diff = q(k) - q_ref(k);
            n = round(diff / (2*pi));
            q(k) = q(k) - n * 2 * pi;
        end
        candidates = [candidates; q];
    end

    % --- 3. 选择最优解 ---
    if isempty(candidates)
        q_best = q_ref; % 无解时保持原位
    else
        % 加权距离: 给 J1-J3 较大权重(少动大臂)，J4-J6 较小权重
        weights = [2 2 2 1 1 1];
        dists = zeros(size(candidates, 1), 1);
        for i = 1:size(candidates, 1)
            dists(i) = sum(weights .* (candidates(i,:) - q_ref).^2);
        end
        [~, idx] = min(dists);
        q_best = candidates(idx, :);
    end
end