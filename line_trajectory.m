function [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = line_trajectory(robot, start_point, end_point, ...
    orientation, total_time, qlim, num_points, q_last)
% LINE_TRAJECTORY 笛卡尔空间直线轨迹规划（精简版）
% 输入:
%   robot - 机器人对象
%   start_point - 起点 [x,y,z]
%   end_point - 终点 [x,y,z]
%   orientation - 姿态 [roll,pitch,yaw]
%   total_time - 总时间
%   qlim - 关节限制
%   num_points - 采样点数, num_points = total*100+1
%   q_last - 上一断轨迹末端的关节角度
%
% 输出:
%   q_traj - 关节轨迹 (M×DOF)
%   time_points - 时间点 (M×1)
%   ee_positions - 末端位置 (M×3)


    % 创建起点和终点的齐次变换矩阵
    T_start = transl(start_point) * rpy2tr(orientation);
    T_goal = transl(end_point) * rpy2tr(orientation);
    
    % 时间采样
    time_points = linspace(0, total_time, num_points)';

    % 使用ctraj进行笛卡尔空间直线插值
    T_traj = ctraj(T_start, T_goal, num_points);

    % 计算逆运动学
    q_traj = zeros(num_points, robot.n);
    q_traj(1,:) = q_last;

    for i = 2:num_points
        q_traj(i,:) = robot.ikine(T_traj(:,:,i), 'q0', q_traj(i-1,:), 'mask', [1 1 1 1 1 1]);

        % 如果求解失败，使用上一个有效解
        if any(isnan(q_traj(i,:)))
            q_traj(i,:) = q_traj(i-1,:);
            warning('第 %d 个点的逆运动学求解失败，使用前一个点的解', i);
        end
    end

    % 计算关节速度和加速度
    qd_traj  = zeros(num_points, robot.n);
    qdd_traj = zeros(num_points, robot.n);
    for j = 1:robot.n
        % 1. 对第 j 个关节拟合三次样条
        pp_q = spline(time_points, q_traj(:,j));

        % 2. 一阶、二阶导数
        pp_qd  = fnder(pp_q, 1);
        pp_qdd = fnder(pp_q, 2);

        % 3. 在原时间点求值
        qd_traj(:,j)  = ppval(pp_qd,  time_points);
        qdd_traj(:,j) = ppval(pp_qdd, time_points);
    end


    % 计算末端位置
    ee_positions = zeros(num_points, 3);
    for i = 1:num_points
        T = robot.fkine(q_traj(i,:));
        ee_positions(i,:) = T.t;
    end
end
