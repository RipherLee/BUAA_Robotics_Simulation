function [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = ptp_trajectory(robot, start_point, end_point, ...
    orientation, total_time, qlim, num_points, q_last, interp_method, v0, vf, a0, af)
% PTP_TRAJECTORY 关节空间点到点轨迹规划
% 
%   v0, vf: 起点和终点的关节速度 (1x6 向量)，默认为 0
%   a0, af: 起点和终点的关节加速度 (1x6 向量)，默认为 0

    % --- 参数预处理 ---
    if nargin < 9, interp_method = 'quintic'; end
    
    % 默认边界条件为静止 (Zero boundary conditions)
    if nargin < 10 || isempty(v0), v0 = zeros(1, robot.n); end
    if nargin < 11 || isempty(vf), vf = zeros(1, robot.n); end
    if nargin < 12 || isempty(a0), a0 = zeros(1, robot.n); end
    if nargin < 13 || isempty(af), af = zeros(1, robot.n); end

    % --- 1. 计算终点关节角 ---
    % 构造目标位姿矩阵
    T_goal = transl(end_point) * rpy2tr(orientation);
    
    % 逆运动学求解 (使用 q_last 作为初值，确保连贯性)
    % 这里优先使用数值解以获得最平滑结果，若失败可回退到外部处理
    try
        q_goal = robot.ikine(T_goal, 'q0', q_last, 'mask', [1 1 1 1 1 1], 'tol', 1e-6);
    catch
        % 如果数值解失败（如奇异点），尝试解析解（如果存在）
        if exist('my_ikine_epson', 'file')
            q_goal = my_ikine_epson(T_goal, q_last);
        else
            warning('PTP规划: IK求解失败，使用 q_last 代替目标');
            q_goal = q_last;
        end
    end
    
    % 自动去卷绕 (解决 +/- 360 度跳变)
    for j=1:robot.n
        q_goal(j) = q_last(j) + wrapToPi(q_goal(j) - q_last(j));
    end
    
    % --- 2. 时间采样 ---
    time_points = linspace(0, total_time, num_points)';
    
    % --- 3. 轨迹规划算法 ---
    q_start = q_last;
    
    switch interp_method
        case 'cubic'
            % 三次多项式 (仅支持速度边界，忽略加速度)
            [q_traj, qd_traj, qdd_traj] = cubic_trajectory(q_start, q_goal, v0, vf, time_points, total_time);
            
        case 'quintic'
            % 五次多项式 (支持位置、速度、加速度全边界)
            [q_traj, qd_traj, qdd_traj] = quintic_trajectory(q_start, q_goal, v0, vf, a0, af, time_points, total_time);
            
        case 's'
            % S曲线 (梯形速度)，通常内部计算最大加减速，暂不支持显式指定边界v/a
            [q_traj, qd_traj, qdd_traj] = s_curve_trajectory(q_start, q_goal, time_points, total_time);
            
        otherwise
            error('未知的插值方法');
    end
    
    % --- 4. 计算末端位置 (FK) ---
    ee_positions = zeros(num_points, 3);
    for i = 1:num_points
        T = robot.fkine(q_traj(i,:));
        ee_positions(i,:) = T.t;
    end
end

%% === 子函数：三次多项式 (Cubic) ===
function [q, dq, ddq] = cubic_trajectory(q0, qf, v0, vf, t, tf)
    n = length(q0); N = length(t);
    q = zeros(N, n); dq = zeros(N, n); ddq = zeros(N, n);
    
    % q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    % 边界条件: q(0)=q0, q'(0)=v0, q(tf)=qf, q'(tf)=vf
    
    for i = 1:n
        a0 = q0(i);
        a1 = v0(i);
        a2 = (3*(qf(i) - q0(i)) - (2*v0(i) + vf(i))*tf) / (tf^2);
        a3 = (2*(q0(i) - qf(i)) + (v0(i) + vf(i))*tf) / (tf^3);
        
        q(:,i)   = a0 + a1.*t + a2.*t.^2 + a3.*t.^3;
        dq(:,i)  = a1 + 2*a2.*t + 3*a3.*t.^2;
        ddq(:,i) = 2*a2 + 6*a3.*t;
    end
end

%% === 子函数：五次多项式 (Quintic - 全边界支持) ===
function [q, dq, ddq] = quintic_trajectory(q0, qf, v0, vf, a0, af, t, tf)
    n = length(q0); N = length(t);
    q = zeros(N, n); dq = zeros(N, n); ddq = zeros(N, n);
    
    % q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    
    for i = 1:n
        % 求解系数 (标准五次多项式公式)
        A = [1,  0,  0,  0,   0,    0;
             0,  1,  0,  0,   0,    0;
             0,  0,  2,  0,   0,    0;
             1, tf, tf^2, tf^3, tf^4, tf^5;
             0,  1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
             0,  0,  2,  6*tf, 12*tf^2, 20*tf^3];
         
        b = [q0(i); v0(i); a0(i); qf(i); vf(i); af(i)];
        
        coeffs = A \ b; % 解线性方程组 [a0; a1; ... a5]
        
        a = coeffs;
        
        % 计算轨迹
        q(:,i)   = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;
        dq(:,i)  = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
        ddq(:,i) = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
    end
end

%% === 子函数：S曲线 (S-Curve - 保持原样) ===
function [q, dq, ddq] = s_curve_trajectory(q0, qf, t, t_total)
    n = length(q0); N = length(t);
    q = zeros(N, n); dq = zeros(N, n); ddq = zeros(N, n);
    t_acc = t_total/3; t_const = t_total/3;
    for i = 1:n
        delta = qf(i)-q0(i);
        a = delta / (t_acc * (t_acc + t_const));
        v_max = a * t_acc;
        for j = 1:N
            tc = t(j);
            if tc <= t_acc
                q(j,i) = q0(i) + 0.5*a*tc^2; dq(j,i) = a*tc; ddq(j,i) = a;
            elseif tc <= t_acc+t_const
                q(j,i) = q0(i) + 0.5*a*t_acc^2 + v_max*(tc-t_acc); dq(j,i) = v_max; ddq(j,i) = 0;
            else
                td = tc - (t_acc+t_const);
                q(j,i) = q0(i) + 0.5*a*t_acc^2 + v_max*t_const + v_max*td - 0.5*a*td^2;
                dq(j,i) = v_max - a*td; ddq(j,i) = -a;
            end
        end
    end
end