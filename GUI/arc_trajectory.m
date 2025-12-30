function [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = arc_trajectory(robot, start_point, via_point, end_point, ...
    orientation_ignored, total_time, qlim, num_points, q_last)
% ARC_TRAJECTORY 圆弧轨迹规划 (矢量旋转修复版)
% 
% 修复原理：
% 1. 摒弃 atan2 角度法，改用罗德里格旋转公式 (Rodrigues' rotation)。
%    这保证了轨迹严格从 start_point 出发，消除了"偏置"。
% 2. 姿态策略改为 "Constant" (保持起点姿态)，防止末端乱转。

    fprintf('  圆弧规划 (矢量法): S[%.2f %.2f %.2f] -> V[%.2f %.2f %.2f] -> E[%.2f %.2f %.2f]\n', ...
        start_point, via_point, end_point);

    % --- 1. 几何计算 (三点定圆) ---
    [center, normal, theta_total, radius] = solve_circle_geometry(start_point, via_point, end_point);
    
    % --- 2. 生成笛卡尔路径点 ---
    % 定义从圆心指向起点的向量
    v_start = start_point - center;
    
    arc_points = zeros(num_points, 3);
    theta_step = linspace(0, theta_total, num_points);
    
    % 使用罗德里格旋转公式生成圆弧
    % V_rot = V*cos(t) + (k x V)*sin(t) + k*(k.V)*(1-cos(t))
    % 这里 k 是法向量 normal (单位向量), k.V = 0 (垂直)
    % 简化为: V_new = center + V_start * cos(t) + (normal x V_start) * sin(t)
    
    cross_vec = cross(normal, v_start); % 旋转轴 x 起点向量
    
    for i = 1:num_points
        th = theta_step(i);
        v_rotated = v_start * cos(th) + cross_vec * sin(th);
        arc_points(i, :) = center + v_rotated;
    end
    
    % 强制修正起点和终点 (消除浮点误差)
    arc_points(1, :) = start_point;
    arc_points(end, :) = end_point;

    % --- 3. 姿态处理 (保持起点姿态) ---
    % 很多时候画圆弧不希望姿态乱动，保持 q_last 的姿态最稳
    T_start_curr = robot.fkine(q_last);
    R_const = T_start_curr.R; % 获取当前姿态旋转矩阵
    
    % --- 4. 逆运动学求解 ---
    q_traj = zeros(num_points, robot.n);
    q_traj(1,:) = q_last;
    
    time_points = linspace(0, total_time, num_points)';
    
    for i = 2:num_points
        % 构造位姿: 位置来自圆弧 + 姿态保持不变
        p_current = arc_points(i, :);
        T_target = rt2tr(R_const, p_current');
        
        try
            % 使用上一时刻角度作为初值，数值解迭代
            q_traj(i,:) = robot.ikine(T_target, 'q0', q_traj(i-1,:), 'mask', [1 1 1 1 1 1], 'tol', 1e-6);
        catch
            % 简单的出错回退
            q_traj(i,:) = q_traj(i-1,:); 
        end
    end
    
    % --- 5. 计算速度加速度 (三次样条) ---
    qd_traj  = zeros(num_points, robot.n);
    qdd_traj = zeros(num_points, robot.n);
    for j = 1:robot.n
        pp = spline(time_points, q_traj(:,j));
        pp_d = fnder(pp, 1);
        pp_dd = fnder(pp, 2);
        qd_traj(:,j) = ppval(pp_d, time_points);
        qdd_traj(:,j) = ppval(pp_dd, time_points);
    end
    
    % 验证输出
    ee_positions = arc_points; 
end

function [center, normal, theta_total, radius] = solve_circle_geometry(P1, P2, P3)
    % 辅助函数：计算圆心、法向量和总转角
    % P1=Start, P2=Via, P3=End (均为 1x3 向量)
    
    v12 = P2 - P1;
    v13 = P3 - P1;
    
    % 法向量 (确定旋转平面)
    normal = cross(v12, v13);
    if norm(normal) < 1e-6
        error('三点共线，无法构成圆弧');
    end
    normal = normal / norm(normal);
    
    % 垂直平分线法求圆心
    % Midpoints
    M12 = (P1 + P2) / 2;
    M23 = (P2 + P3) / 2;
    
    % 向量: M12 -> Center 必须垂直于 v12，且在平面内
    % dir12 = cross(v12, normal)
    dir12 = cross(v12, normal);
    dir23 = cross(P3 - P2, normal);
    
    % 解交点: M12 + a*dir12 = M23 + b*dir23
    % a*dir12 - b*dir23 = M23 - M12
    rhs = (M23 - M12)';
    lhs = [dir12', -dir23'];
    
    % 使用伪逆求解 (2x2 线性方程)
    params = lhs \ rhs;
    a = params(1);
    
    center = M12 + a * dir12;
    radius = norm(P1 - center);
    
    % --- 计算角度范围 ---
    % 定义局部坐标系向量
    u = P1 - center; 
    v = P2 - center;
    w = P3 - center;
    
    % 计算 Start -> Via 的角度
    % 使用带符号角度计算 angle(u, v, normal)
    ang_sv = atan2(dot(cross(u, v), normal), dot(u, v));
    
    % 计算 Via -> End 的角度
    ang_ve = atan2(dot(cross(v, w), normal), dot(v, w));
    
    % 保证角度是正向累加的 (顺着 Start->Via->End 的方向)
    if ang_sv < 0, ang_sv = ang_sv + 2*pi; end
    if ang_ve < 0, ang_ve = ang_ve + 2*pi; end
    
    theta_total = ang_sv + ang_ve;
end