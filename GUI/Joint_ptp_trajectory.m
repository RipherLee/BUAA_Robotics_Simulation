function [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = ptp_trajectory(robot, start_point, end_point, ...
    orientation, total_time, qlim, num_points, q_last, interp_method)
% PTP_TRAJECTORY 关节空间点到点轨迹规划
% 输入:
%   robot - 机器人对象
%   start_point - 起点 [x,y,z]
%   end_point - 终点 [x,y,z]
%   orientation - 姿态 [roll,pitch,yaw]
%   total_time - 总时间
%   qlim - 关节限制
%   num_points - 采样点数, num_points = total*100+1
%   q_last - 上一断轨迹末端的关节角度(单独一段轨迹的话可以赋值为[0 0 0 0 0 0])
%   interp_method - 插值方法: 'cubic'(三次插值), 'quintic'(五次插值), 's'(S曲线)
%
% 输出:
%   q_traj - 关节轨迹 (M×DOF)
%   qd_traj - 关节速度轨迹 (M×DOF)
%   qdd_traj - 关节加速度轨迹 (M×DOF)
%   time_points - 时间点 (M×1)
%   ee_positions - 末端位置 (M×3)

    % % 验证输入参数
    % if nargin < 9
    %     error('需要指定插值方法: ''cubic'', ''quintic'', 或 ''s''');
    % end
    
    % 验证插值方法
    valid_methods = {'cubic', 'quintic', 's'};
    if ~ismember(interp_method, valid_methods)
        error('插值方法必须是 ''cubic'', ''quintic'', 或 ''s''');
    end

    % 创建起点和终点的齐次变换矩阵
    T_start = transl(start_point) * rpy2tr(orientation);
    T_goal = transl(end_point) * rpy2tr(orientation);
    
    % 使用逆运动学求解起始点和目标点关节角度
    q_start = q_last;
    q_goal = robot.ikine(T_goal, 'q0', q_start, 'mask', [1 1 1 1 1 1]);
    
    % 检查是否成功求解逆运动学
    if any(isnan(q_start)) || any(isnan(q_goal))
        error('逆运动学求解失败！请检查目标位姿是否在机器人的工作空间内。');
    end
    
    % 检查关节限制
    n = length(q_start); % 关节数
    for i = 1:n
        if q_start(i) < qlim(i,1) || q_start(i) > qlim(i,2) || ...
           q_goal(i) < qlim(i,1) || q_goal(i) > qlim(i,2)
            warning('关节角度超出限制！关节 %d: 限制范围 [%.2f, %.2f], 起点: %.2f, 终点: %.2f', ...
                i, qlim(i,1), qlim(i,2), q_start(i), q_goal(i));
        end
    end
    
    % 时间采样
    time_points = linspace(0, total_time, num_points)';
    
    % 根据选择的插值方法进行轨迹规划
    switch interp_method
        case 'cubic'
            [q_traj, qd_traj, qdd_traj] = cubic_trajectory(q_start, q_goal, time_points, total_time);
        case 'quintic'
            [q_traj, qd_traj, qdd_traj] = quintic_trajectory(q_start, q_goal, time_points, total_time);
        case 's'
            [q_traj, qd_traj, qdd_traj] = s_curve_trajectory(q_start, q_goal, time_points, total_time);
    end
    
    % 计算末端位置
    ee_positions = zeros(num_points, 3);
    for i = 1:num_points
        T = robot.fkine(q_traj(i,:));
        ee_positions(i,:) = T.t;
    end
end

% 三次多项式轨迹规划子函数
function [q, dq, ddq] = cubic_trajectory(q0, qf, t, t_total)
    n = length(q0); % 关节数
    N = length(t);  % 轨迹点数量
    
    q = zeros(N, n);
    dq = zeros(N, n);
    ddq = zeros(N, n);
    
    for i = 1:n
        % 计算三次多项式系数
        a0 = q0(i);
        a1 = 0;  % 初始速度为0
        a2 = 3/(t_total^2) * (qf(i) - q0(i));
        a3 = -2/(t_total^3) * (qf(i) - q0(i));
        
        % 计算轨迹
        for j = 1:N
            t_curr = t(j);
            q(j,i) = a0 + a1*t_curr + a2*t_curr^2 + a3*t_curr^3;
            dq(j,i) = a1 + 2*a2*t_curr + 3*a3*t_curr^2;
            ddq(j,i) = 2*a2 + 6*a3*t_curr;
        end
    end
end

% 五次多项式轨迹规划子函数
function [q, dq, ddq] = quintic_trajectory(q0, qf, t, t_total)
    n = length(q0); % 关节数
    N = length(t);  % 轨迹点数量
    
    q = zeros(N, n);
    dq = zeros(N, n);
    ddq = zeros(N, n);
    
    for i = 1:n
        % 计算五次多项式系数
        a0 = q0(i);
        a1 = 0;  % 初始速度为0
        a2 = 0;  % 初始加速度为0
        a3 = 10/(t_total^3) * (qf(i) - q0(i));
        a4 = -15/(t_total^4) * (qf(i) - q0(i));
        a5 = 6/(t_total^5) * (qf(i) - q0(i));
        
        % 计算轨迹
        for j = 1:N
            t_curr = t(j);
            q(j,i) = a0 + a1*t_curr + a2*t_curr^2 + a3*t_curr^3 + ...
                     a4*t_curr^4 + a5*t_curr^5;
            dq(j,i) = a1 + 2*a2*t_curr + 3*a3*t_curr^2 + ...
                     4*a4*t_curr^3 + 5*a5*t_curr^4;
            ddq(j,i) = 2*a2 + 6*a3*t_curr + 12*a4*t_curr^2 + 20*a5*t_curr^3;
        end
    end
end

% S曲线轨迹规划子函数
function [q, dq, ddq] = s_curve_trajectory(q0, qf, t, t_total)
    n = length(q0); % 关节数
    N = length(t);  % 轨迹点数量
    
    q = zeros(N, n);
    dq = zeros(N, n);
    ddq = zeros(N, n);
    
    % 对称的S曲线：加速段、匀速段、减速段时间各占1/3
    t_acc = t_total / 3;
    t_const = t_total / 3;
    
    for i = 1:n
        % 计算位移
        delta_q = qf(i) - q0(i);
        
        % 计算加速度a
        % 总位移 = 加速段位移 + 匀速段位移 + 减速段位移
        % delta_q = 0.5*a*t_acc^2 + (a*t_acc)*t_const + 0.5*a*t_acc^2
        % delta_q = a*t_acc*(t_acc + t_const)
        if t_acc > 0
            a = delta_q / (t_acc * (t_acc + t_const));
        else
            a = 0;
        end
        
        % 计算最大速度
        v_max = a * t_acc;
        
        % 计算轨迹
        for j = 1:N
            t_curr = t(j);
            if t_curr <= t_acc
                % 匀加速段
                q(j,i) = q0(i) + 0.5*a*t_curr^2;
                dq(j,i) = a*t_curr;
                ddq(j,i) = a;
            elseif t_curr <= (t_acc + t_const)
                % 匀速段
                q(j,i) = q0(i) + 0.5*a*t_acc^2 + v_max*(t_curr - t_acc);
                dq(j,i) = v_max;
                ddq(j,i) = 0;
            elseif t_curr <= t_total
                % 匀减速段
                t_dec = t_curr - (t_acc + t_const);
                q(j,i) = q0(i) + 0.5*a*t_acc^2 + v_max*t_const + ...
                        v_max*t_dec - 0.5*a*t_dec^2;
                dq(j,i) = v_max - a*t_dec;
                ddq(j,i) = -a;
            else
                % 超过总时间，保持在终点
                q(j,i) = qf(i);
                dq(j,i) = 0;
                ddq(j,i) = 0;
            end
        end
    end
end