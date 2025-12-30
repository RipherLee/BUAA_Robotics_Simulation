%% 主程序：机器人多点轨迹规划
clear; clc; close all;

%% 创建机器人模型
L_M(1) = Link('revolute', 'd', 0.12202, 'a', 0, 'alpha', 0, 'modified');
L_M(2) = Link('revolute', 'd', 0, 'a', 0.1, 'alpha', -pi/2, 'modified', 'offset', -pi/2);
L_M(3) = Link('revolute', 'd', 0, 'a', 0.25, 'alpha', 0, 'modified', 'offset', pi);
L_M(4) = Link('revolute', 'd', 0.187+0.063, 'a', 0, 'alpha', pi/2, 'modified');
L_M(5) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'modified');
L_M(6) = Link('revolute', 'd', 0.0595, 'a', 0, 'alpha', pi/2, 'modified');
robot = SerialLink(L_M, 'name', 'EpsonC3_M', 'base', transl(0, 0, 0.198));

%% 设置工作空间和关节限制
workspace = [-0.5 0.5 -0.5 0.5 0 0.7];
qlim = [
    -170/180*pi, 170/180*pi;      -160/180*pi, 65/180*pi;    -51/180*pi, 225/180*pi;
    -200/180*pi, 200/180*pi;      -135/180*pi, 135/180*pi;   -360/180*pi, 360/180*pi;
];

%% 定义多点路径和移动类型
% 路径点 [x, y, z]
path_points = {
    [0.4095, 0.0000, 0.5700];    % 点1(初始状态，关节角均为零)
    [0.35, -0.10, 0.65];    [0.55, 0.15, 0.40];   
    % 点2 (物体1)            % 点3 (放置点1)
    [0.35, -0.15, 0.65];    [0.50, 0.15, 0.40];    
    % 点4 (物体2)            % 点5 (放置点2)
    [0.35, -0.20, 0.65];    [0.45, 0.15, 0.40];   
    % 点6 (物体3)            % 点7 (放置点3)
    [0.35, -0.10, 0.60];    [0.55, 0.10, 0.40];     
    % 点8 (物体4)            % 点9 (放置点4)
    [0.35, -0.15, 0.60];    [0.50, 0.10, 0.40];    
    % 点10 (物体5)           % 点11 (放置点5)
    [0.35, -0.20, 0.60];    [0.45, 0.10, 0.40];     
    % 点12 (物体6)           % 点13 (放置点6)
};

% 移动类型: 'PTP', 'LINE', 'ARC'
path_types = {'LINE', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP',...
              'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP'};

% 姿态配置（可以为每段路径指定不同的姿态，but未写）
% 如果为1×3，所有点使用相同姿态；如果为N×3，每点使用对应姿态
orientation = [0, 0, 0];  % 固定姿态

% 每段路径时间（秒）
segment_times = [2.0, 3.0, 3.0, 3.0, 3.0, 3.0,...
                 3.0, 3.0, 3.0, 3.0, 3.0, 3.0];

%% 执行多点轨迹规划
fprintf('多点轨迹规划开始...\n');
fprintf('总共有 %d 个路径点\n', length(path_points));

all_q_traj = [];
all_qd_traj = [];
all_qdd_traj = [];
all_time = [];
all_ee_positions = [];
current_time = 0;

% 初始化关节角
q_last = [0 0 0 0 0 0];
for i = 1:length(path_types)
    fprintf('\n--- 第 %d 段: 从点%d到点%d, 类型: %s, 时间: %.1f秒 ---\n', ...
            i, i, mod(i, length(path_points))+1, path_types{i}, segment_times(i));
    
    % 获取起点、终点
    start_point = path_points{i};
    end_point = path_points{mod(i, length(path_points))+1};

    % 根据每段的时间计算采样点数
    num_points = segment_times(i)*100+1; % 3秒则采样301个点，2秒则采样201个点(dt = 0.01)

    % 根据路径类型调用相应的轨迹规划函数
    switch path_types{i}
        case 'PTP'
            % PTP轨迹规划
            [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = ptp_trajectory(...
                robot, start_point, end_point, orientation, ...
                segment_times(i), qlim, num_points, q_last);
            
        case 'LINE'
            % 直线轨迹规划
            [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = line_trajectory(...
                robot, start_point, end_point, orientation, ...
                segment_times(i), qlim, num_points, q_last);
            
        case 'ARC'
            % 圆弧轨迹规划（需要一个中间点）
            % via_point = zeros(1,3);
            % via_point = choose_third_point(start_point, end_point, 0.05)
            % via_point = [0.4, 0, 0.5]

            [q_traj, qd_traj, qdd_traj, time_points, ee_positions] = arc_trajectory(...
                robot, start_point, via_point, end_point, ...
                orientation, segment_times(i), qlim, num_points, q_last);
    end
    q_last = q_traj(end,:);
    % qd_last = qd_traj(end,:);

    % 调整时间（加上之前的时间）
    time_points = time_points + current_time;
    current_time = time_points(end);
    
    % 拼接轨迹
    if i == 1
        all_q_traj = q_traj;
        all_qd_traj = qd_traj;
        all_qdd_traj = qdd_traj;
        all_time = time_points;
        all_ee_positions = ee_positions;
    else
        % 去掉第一个点（与上一段的最后一个点重复）
        all_q_traj = [all_q_traj; q_traj(2:end, :)];
        all_qd_traj = [all_qd_traj; qd_traj(2:end, :)];
        all_qdd_traj = [all_qdd_traj; qdd_traj(2:end, :)];
        all_time = [all_time; time_points(2:end)];
        all_ee_positions = [all_ee_positions; ee_positions(2:end, :)];
    end
end

fprintf('\n=== 轨迹规划完成 ===\n');
fprintf('总时间: %.2f 秒\n', all_time(end));
fprintf('总轨迹点数: %d\n', size(all_q_traj, 1));

%% 保存整个过程机器人的关节角,关节角速度，关节角速度，末端位置随时间的变化
% 创建表格
T = table(all_time, ...
          all_q_traj(:,1), all_q_traj(:,2), all_q_traj(:,3), ...
          all_q_traj(:,4), all_q_traj(:,5), all_q_traj(:,6), ...
          all_qd_traj(:,1), all_qd_traj(:,2), all_qd_traj(:,3), ...
          all_qd_traj(:,4), all_qd_traj(:,5), all_qd_traj(:,6), ...
          all_qdd_traj(:,1), all_qdd_traj(:,2), all_qdd_traj(:,3), ...
          all_qdd_traj(:,4), all_qdd_traj(:,5), all_qdd_traj(:,6), ...
          all_ee_positions(:,1), all_ee_positions(:,2), all_ee_positions(:,3), ...
          'VariableNames', {...
               'time', ...
               'q1','q2','q3','q4','q5','q6', ...
               'qd1','qd2','qd3','qd4','qd5','qd6', ...
               'qdd1','qdd2','qdd3','qdd4','qdd5','qdd6', ...
               'x','y','z'});

% 保存 CSV
writetable(T, 'robot_trajectory_table.csv');

%% 展示机器人动画和末端三维轨迹
fprintf('\n开始展示机器人动画和末端轨迹...\n');

% 末端三维轨迹
figure('Position', [50, 50, 800, 600], 'Name', '末端三维轨迹');
hold on;
% 用不同颜色绘制每段轨迹
colors = {'r', 'g', 'b', 'm', 'c', 'y'};
current_idx = 1;
for i = 1:length(path_types)
    % 计算当前段的点数（近似）
    segment_points = round(size(all_q_traj, 1) / length(path_types));
    if i == length(path_types)
        end_idx = size(all_ee_positions, 1);
    else
        end_idx = min(current_idx + segment_points - 1, size(all_ee_positions, 1));
    end
    
    % 绘制当前段轨迹
    plot3(all_ee_positions(current_idx:end_idx, 1), ...
          all_ee_positions(current_idx:end_idx, 2), ...
          all_ee_positions(current_idx:end_idx, 3), ...
          'LineWidth', 2, 'Color', colors{mod(i-1,6)+1}, ...
          'DisplayName', sprintf('段%d: %s', i, path_types{i}));
    
    current_idx = end_idx;
end

% 绘制路径点
for i = 1:length(path_points)
    plot3(path_points{i}(1), path_points{i}(2), path_points{i}(3), ...
          'o', 'MarkerSize', 10, 'MarkerFaceColor', colors{mod(i-1,6)+1}, ...
          'MarkerEdgeColor', 'k', 'LineWidth', 2);
    text(path_points{i}(1)+0.02, path_points{i}(2)+0.02, path_points{i}(3)+0.02, ...
         sprintf('P%d', i), 'FontSize', 12, 'FontWeight', 'bold');
end

grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('末端三维轨迹');
legend('Location', 'best', 'FontSize', 8);
view(45,30);
axis equal;

%% ===== 关节角 / 速度 / 加速度（同一图，三个子图）=====
figure('Name','关节轨迹总览','Position',[100 100 1000 750]);

joint_names = {'q1','q2','q3','q4','q5','q6'};

% ---------- 1. 关节角 ----------
subplot(3,1,1);
hold on; grid on;
for j = 1:6
    plot(all_time, all_q_traj(:,j), 'LineWidth', 1.5);
end
ylabel('角度 (rad)');
title('关节角度 q(t)');
legend(joint_names, 'Location','best');
% ---------- 2. 关节速度 ----------
subplot(3,1,2);
hold on; grid on;
for j = 1:6
    plot(all_time, all_qd_traj(:,j), 'LineWidth', 1.5);
end
ylabel('速度 (rad/s)');
title('关节速度 \dot{q}(t)');
% ---------- 3. 关节加速度 ----------
subplot(3,1,3);
hold on; grid on;
for j = 1:6
    plot(all_time, all_qdd_traj(:,j), 'LineWidth', 1.5);
end
xlabel('时间 (s)');
ylabel('加速度 (rad/s^2)');
title('关节加速度 \ddot{q}(t)');



% 机器人动画
figure('Position', [100, 100, 800, 600], 'Name', '机器人多点轨迹演示');
robot.plot(all_q_traj(1,:), 'workspace', workspace, 'trail', 'r-', 'fps', 30);
hold on;

% 绘制路径点
for i = 1:length(path_points)
    plot3(path_points{i}(1), path_points{i}(2), path_points{i}(3), ...
          'o', 'MarkerSize', 8, 'MarkerFaceColor', colors{mod(i-1,6)+1}, ...
          'MarkerEdgeColor', 'k', 'LineWidth', 1);
end

% 绘制末端轨迹
plot3(all_ee_positions(:,1), all_ee_positions(:,2), all_ee_positions(:,3), ...
      'b-', 'LineWidth', 2, 'DisplayName', '末端轨迹');

grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('机器人动画演示');
view(45,30);
axis equal;

% 图例和路径类型标注
legend_str = cell(1, length(path_points));
for i = 1:length(path_points)
    legend_str{i} = sprintf('P%d (%s)', i, path_types{mod(i-1, length(path_types))+1});
    text(path_points{i}(1)+0.02, path_points{i}(2)+0.02, path_points{i}(3)+0.02, ...
         sprintf('P%d', i), 'FontSize', 10, 'FontWeight', 'bold');
end
legend('末端轨迹', 'Location', 'best');

%% 播放动画
fprintf('正在播放动画...\n');
pause(1);

% 播放机器人动画
for i = 1:5:size(all_q_traj, 1)  % 跳帧以加快播放速度
    robot.animate(all_q_traj(i,:));
    drawnow;
end

% 确保显示最终位置
robot.animate(all_q_traj(end,:));
drawnow;

fprintf('\n演示完成！\n');
fprintf('轨迹类型统计:\n');
for i = 1:length(path_types)
    fprintf('  段%d: %s (%.1f秒)\n', i, path_types{i}, segment_times(i));
end

% function p3 = choose_third_point(A, B, s)
% %CHOOSE_THIRD_POINT  为两点 A,B 选择圆弧的第三点 p3（均为行向量）
% %   p3 = choose_third_point(A, B, s)
% %
% % 输入：
% %   A, B : 1×3 行向量
% %   s    : 偏移量（可选），控制弯曲程度
% %          默认 s = 0.1 * |B-A|
% %
% % 输出：
% %   p3   : 1×3 行向量形式的第三点
% %
% % 原理：
% %   p3 = M + s * u
% %   M 为 A,B 中点；u 为与 (B-A) 垂直的单位向量
% 
%     % 保证为列向量用于内部计算
%     A = A(:);
%     B = B(:);
% 
%     % 若没有给 s，默认取弦长 10%
%     if nargin < 3
%         s = 0.1 * norm(B - A);
%     end
% 
%     % 弦向量
%     d = B - A;
%     nd = norm(d);
%     if nd < 1e-9
%         error('A 与 B 太近，无法确定第三点');
%     end
%     d_unit = d / nd;
% 
%     % 中点
%     M = (A + B) / 2;
% 
%     % 选择一个不共线的参考向量 e
%     if abs(dot(d_unit, [0;0;1])) > 0.99
%         e = [1;0;0];
%     else
%         e = [0;0;1];
%     end
% 
%     % 与 d 垂直的方向 u
%     u = cross(d, e);
%     nu = norm(u);
% 
%     if nu < 1e-9
%         % 极端情况：d 与 e 平行，再换一个 e
%         e = [0;1;0];
%         u = cross(d, e);
%         nu = norm(u);
%     end
% 
%     u = u / nu;
% 
%     % 计算第三点（列向量）
%     p3_col = M + s * u;
% 
%     % 转换成行向量输出
%     p3 = p3_col';
% end
