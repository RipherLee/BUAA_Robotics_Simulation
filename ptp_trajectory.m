function [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = ptp_trajectory(robot, start_point, end_point, ...
    orientation, total_time, qlim, num_points, q_last)
% PTP_TRAJECTORY 关节空间点到点轨迹规划（精简版）
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
    
    % 使用逆运动学求解起始点和目标点关节角度
    q_start = q_last;
    q_goal = robot.ikine(T_goal, 'q0', q_start, 'mask', [1 1 1 1 1 1]);
    
    % 检查是否成功求解逆运动学
    if any(isnan(q_start)) || any(isnan(q_goal))
        error('逆运动学求解失败！请检查目标位姿是否在机器人的工作空间内。');
    end
    
    % 时间采样
    time_points = linspace(0, total_time, num_points)';
    
    % 使用jtraj函数进行五次多项式插值（保证速度、加速度连续）
    [q_traj, qd_traj, qdd_traj] = jtraj(q_start, q_goal, time_points);
    
    % 计算末端位置
    ee_positions = zeros(num_points, 3);
    for i = 1:num_points
        T = robot.fkine(q_traj(i,:));
        ee_positions(i,:) = T.t;
    end
end