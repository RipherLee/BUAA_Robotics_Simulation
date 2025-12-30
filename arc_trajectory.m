function [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = arc_trajectory(robot, start_point, via_point, end_point, ...
    orientation, total_time, qlim, num_points, q_last)
% ARC_TRAJECTORY 圆弧轨迹规划
% 输入:
%   robot - 机器人对象
%   start_point - 起点 [x,y,z]
%   via_point - 中间点 [x,y,z]
%   end_point - 终点 [x,y,z]
%   orientation - 姿态 [roll,pitch,yaw] 或 N×3矩阵
%   total_time - 总时间
%   qlim - 关节限制
%   num_points - 采样点数, num_points = total*100+1
%   q_last - 上一断轨迹末端的关节角度
%
% 输出:
%   q_smooth - 平滑关节轨迹 (M×DOF)
%   time_points - 时间点 (M×1)
%   ee_positions - 末端位置 (M×3)

    
    fprintf('  圆弧轨迹规划: 起点[%.3f,%.3f,%.3f] -> 终点[%.3f,%.3f,%.3f]\n', ...
            start_point(1), start_point(2), start_point(3), ...
            end_point(1), end_point(2), end_point(3));
    
    % 生成圆弧路径点
    arc_points = generate_arc(start_point, via_point, end_point, num_points);
    
    % 时间点
    time_points = linspace(0, total_time, num_points)';
    
    % 规划关节轨迹
    q_traj = plan_joint_trajectory(robot, arc_points, orientation, time_points, qlim, q_last);
    
    % 计算关节速度和加速度
    qd_traj  = zeros(num_points, robot.n);
    qdd_traj = zeros(num_points, robot.n);
    for j = 1:robot.n
        % 对第 j 个关节进行样条拟合
        pp_q = spline(time_points, q_traj(:,j));

        % 一阶、二阶导
        pp_qd  = fnder(pp_q, 1);
        pp_qdd = fnder(pp_q, 2);

        % 在原时间点求值
        qd_traj(:,j)  = ppval(pp_qd,  time_points);
        qdd_traj(:,j) = ppval(pp_qdd, time_points);
    end


    % 计算末端位置
    ee_positions = zeros(size(q_traj, 1), 3);
    for i = 1:size(q_traj, 1)
        T = robot.fkine(q_traj(i,:));
        ee_positions(i,:) = T.t;
    end
    
    fprintf('  规划完成: 点数=%d, 时间=%.2fs\n', size(q_traj,1), total_time);
end

function [arc_points, arc_length, center, radius] = generate_arc(start_point, via_point, end_point, num_points)
    % 生成圆弧路径点
    % start_point: 起点 [x,y,z]
    % via_point: 中间点 [x,y,z]
    % end_point: 终点 [x,y,z]
    % num_points: 采样点数
    
    % 计算三点确定的圆
    [center, radius, normal_vec] = fit_circle_3d(start_point, via_point, end_point);
    
    % 创建局部坐标系
    if abs(normal_vec(3)) > 0.9
        x_axis = [1; 0; 0];
    else
        x_axis = cross([0;0;1], normal_vec);
        x_axis = x_axis / norm(x_axis);
    end
    y_axis = cross(normal_vec, x_axis);
    
    % 将点转换到局部坐标系
    R = [x_axis, y_axis, normal_vec];
    T = [R, center; 0, 0, 0, 1];
    
    local_start = (T \ [start_point, 1]')';
    local_via = (T \ [via_point, 1]')';
    local_end = (T \ [end_point, 1]')';
    
    % 计算起始角和终止角
    start_angle = atan2(local_start(2), local_start(1));
    via_angle = atan2(local_via(2), local_via(1));
    end_angle = atan2(local_end(2), local_end(1));
    
    % 调整角度连续性
    angles = [start_angle, via_angle, end_angle];
    for i = 2:3
        while angles(i) - angles(i-1) < -pi
            angles(i) = angles(i) + 2*pi;
        end
        while angles(i) - angles(i-1) > pi
            angles(i) = angles(i) - 2*pi;
        end
    end
    
    start_angle = angles(1);
    end_angle = angles(3);
    
    % 生成圆弧参数
    arc_angles = linspace(start_angle, end_angle, num_points);
    arc_points_local = [radius * cos(arc_angles); 
                        radius * sin(arc_angles); 
                        zeros(1, num_points); 
                        ones(1, num_points)];
    
    % 转换回全局坐标系
    arc_points = zeros(num_points, 3);
    for i = 1:num_points
        global_pt = T * arc_points_local(:, i);
        arc_points(i, :) = global_pt(1:3)';
    end
    
    % 计算弧长
    arc_length = radius * abs(end_angle - start_angle);
end

function [center, radius, normal] = fit_circle_3d(p1, p2, p3)
    % 三维空间中三点确定圆
    p1 = p1(:);
    p2 = p2(:);
    p3 = p3(:);

    % 计算三角形的边向量
    v1 = p2 - p1;
    v2 = p3 - p1;

    % 平面法向量
    normal = cross(v1, v2);
    if norm(normal) < eps
        error('三点共线，无法确定唯一的圆');
    end
    normal = normal / norm(normal);

    % 选择适当的局部坐标系
    if abs(normal(3)) > 0.9
        basis1 = [1;0;0];
    else
        basis1 = [0;0;1];
    end
    basis2 = cross(normal, basis1);
    basis2 = basis2 / norm(basis2);
    basis1 = cross(basis2, normal);

    % 投影到2D平面
    p1_2d = [dot(p1, basis1); dot(p1, basis2)];
    p2_2d = [dot(p2, basis1); dot(p2, basis2)];
    p3_2d = [dot(p3, basis1); dot(p3, basis2)];

    % 2D中计算圆心
    A = 2 * [p2_2d(1)-p1_2d(1), p2_2d(2)-p1_2d(2);
             p3_2d(1)-p1_2d(1), p3_2d(2)-p1_2d(2)];
    B = [p2_2d(1)^2-p1_2d(1)^2 + p2_2d(2)^2-p1_2d(2)^2;
         p3_2d(1)^2-p1_2d(1)^2 + p3_2d(2)^2-p1_2d(2)^2];

    center_2d = A \ B;

    % 转换回3D
    center = center_2d(1)*basis1 + center_2d(2)*basis2;
    radius = norm(p1 - center);
end

function q_traj = plan_joint_trajectory(robot, cartesian_points, orientation, time_points, qlim, q_last)
    % 规划关节轨迹
    N = size(cartesian_points, 1);
%     if size(orientation, 1) == 1
%         % 固定姿态
%         orientation = repmat(orientation, N, 1);
%     end
    
    % 计算每个点的逆运动学
    q_traj = zeros(N, robot.n);
    
    % 第一个点使用上一段轨迹末端的角度
    q_traj(1,:) = q_last;
    
    % 后续点使用上一个点作为初始猜测
    for i = 2:N
        T_i = transl(cartesian_points(i,:)) * rpy2tr(orientation);
        q_traj(i,:) = robot.ikine(T_i, 'q0', q_traj(i-1,:), 'mask', [1 1 1 1 1 1]);
        
        % 检查关节限制
        for j = 1:robot.n
            if q_traj(i,j) < qlim(j,1) || q_traj(i,j) > qlim(j,2)
                fprintf('警告: 点%d关节%d超出限制: %.3f rad\n', i, j, q_traj(i,j));
            end
        end
    end
end
