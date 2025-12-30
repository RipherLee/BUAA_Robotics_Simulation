function varargout = GUI(varargin)
% GUI MATLAB code file for GUI.fig
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end
if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
end

function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
    handles.output = hObject;
    % =========================================================================
    % 初始化机器人模型 (Epson C3)
    % =========================================================================
    % 定义连杆长度
    d4_val = 0.187 + 0.063; 

    % 建立 Standard DH 模型
    L(1) = Link('revolute', 'd', 0.12202, 'a', 0.1, 'alpha', -pi/2, 'standard');
    L(2) = Link('revolute', 'd', 0, 'a', 0.25, 'alpha', 0, 'standard', 'offset', -pi/2);
    L(3) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'standard', 'offset', pi);
    L(4) = Link('revolute', 'd', d4_val, 'a', 0, 'alpha', -pi/2, 'standard');
    L(5) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'standard');
    L(6) = Link('revolute', 'd', 0.0595, 'a', 0, 'alpha', 0, 'standard');
    robot = SerialLink(L, 'name', 'Epson C3 A601', 'base', transl(0,0,0.198));
    file_path = '.\EpsonC3A601_SW\';
    
    % 关节限制
    qlim_val = [
        -170, 170; -160, 65; -51, 225;
        -200, 200; -135, 135; -360, 360;
    ] * pi/180;
    robot.qlim = qlim_val;
    
    ws = [-0.6 0.6 -0.6 0.6 0 0.8];
    handles.robot = robot;
    handles.qlim = qlim_val;
    handles.ws = ws;
    handles.file_path = file_path;
    
    % =========================================================================
    % 初始化绘图区
    % =========================================================================
    axes(handles.axes_3d);
    view(45, 30); 
%     light('Position', [1, 1, -1], 'color', 'w'); % 打光，不然机械臂有点暗
%     light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
    robot.plot3d(zeros(1,6), 'workspace', ws, 'scale', 0.6, 'path', file_path, 'nowrist');
    hold(handles.axes_3d, 'on');
    grid(handles.axes_3d, 'on');
    title(handles.axes_3d, '机器人 3D 实时显示');
    
    % 初始化波形图
    axes(handles.axes_q);   grid on; title('关节角度');
    axes(handles.axes_qd);  grid on; title('关节速度');
    axes(handles.axes_qdd); grid on; title('关节加速度');
    
    % =========================================================================
    % 初始化面板
    % =========================================================================
    set(handles.panel_fk, 'Visible', 'on');
    set(handles.panel_traj, 'Visible', 'off');
    set(handles.panel_main, 'Visible', 'off');
    
    % Update handles structure
    guidata(hObject, handles);
end

% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
    varargout{1} = handles.output;
end

% =========================================================================
% 导航栏回调函数
% =========================================================================
function btn_nav_fk_Callback(hObject, eventdata, handles)
    set(handles.btn_nav_fk, 'Value', 1); 
    set(handles.btn_nav_traj, 'Value', 0); 
    set(handles.btn_nav_main, 'Value', 0);
    set(handles.panel_fk, 'Visible', 'on'); 
    set(handles.panel_traj, 'Visible', 'off'); 
    set(handles.panel_main, 'Visible', 'off');
end

function btn_nav_traj_Callback(hObject, eventdata, handles)
    set(handles.btn_nav_fk, 'Value', 0); 
    set(handles.btn_nav_traj, 'Value', 1); 
    set(handles.btn_nav_main, 'Value', 0);
    set(handles.panel_fk, 'Visible', 'off'); 
    set(handles.panel_traj, 'Visible', 'on'); 
    set(handles.panel_main, 'Visible', 'off');
end

function btn_nav_main_Callback(hObject, eventdata, handles)
    set(handles.btn_nav_fk, 'Value', 0); 
    set(handles.btn_nav_traj, 'Value', 0); 
    set(handles.btn_nav_main, 'Value', 1);
    set(handles.panel_fk, 'Visible', 'off'); 
    set(handles.panel_traj, 'Visible', 'off'); 
    set(handles.panel_main, 'Visible', 'on');
end

% =========================================================================
% 运动学控制
% =========================================================================

% 正运动学
function btn_solve_fk_Callback(hObject, eventdata, handles)
    q = zeros(1,6);
    for i=1:6
        val = str2double(get(handles.(sprintf('edit_j%d', i)), 'String'));
        if isnan(val)
            val = 0; 
        end
        q(i) = val * pi/180;
    end
    
    try
        % -----------------------------------------------------
        % 清理并重绘
        axes(handles.axes_3d); 
        cla(handles.axes_3d); % 清除残留图像
        
        
        view(45, 30);
        light('Position', [1, 1, -1], 'color', 'w'); % 打光，不然机械臂有点暗
        light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
        handles.robot.plot3d(q, ...
                             'workspace', handles.ws,  ...
                             'scale',0.6, ...
                             'path', handles.file_path, ...
                             'nowrist'); % 重绘机器人
        hold(handles.axes_3d, 'on'); 
        grid(handles.axes_3d, 'on');
        % -----------------------------------------------------

        T = handles.robot.fkine(q);
        
        % 更新末端显示
        pos = T.t'; 
        rpy = tr2rpy(T, 'deg');
        vals = [pos, rpy];
        for i=1:6
            set(handles.(sprintf('edit_p%d', i)), 'String', sprintf('%.3f', vals(i))); 
        end
        
    catch ME
        errordlg(['FK Error: ' ME.message]);
    end
end

% 逆运动学
function btn_solve_ik_Callback(hObject, eventdata, handles)
    q_curr = zeros(1,6);
    for i=1:6, q_curr(i) = str2double(get(handles.(sprintf('edit_j%d', i)), 'String')) * pi/180; end
    
    % -----------------------------------------------------
    % 计算前清理绘图区，防止残留轨迹
    axes(handles.axes_3d); 
    cla(handles.axes_3d); % 清除残留图像


    view(45, 30);
    light('Position', [1, 1, -1], 'color', 'w'); % 打光，不然机械臂有点暗
    light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
    handles.robot.plot3d(q_curr, ...
                         'workspace', handles.ws,  ...
                         'scale',0.6, ...
                         'path', handles.file_path, ...
                         'nowrist'); % 重绘机器人
    hold(handles.axes_3d, 'on'); 
    grid(handles.axes_3d, 'on');
    % -----------------------------------------------------

    tags_pos = {'edit_p1','edit_p2','edit_p3','edit_p4','edit_p5','edit_p6'};
    target_vals = zeros(1,6);
    for i=1:6, target_vals(i) = str2double(get(handles.(tags_pos{i}), 'String')); end
    T_target = transl(target_vals(1:3)) * rpy2tr(target_vals(4:6), 'deg');
    
    q_sol = [];
    success = false;
    
    T_curr = handles.robot.fkine(q_curr);
    pos_curr = T_curr.t';
    rpy_curr = tr2rpy(T_curr, 'deg');
    vals_curr = [pos_curr, rpy_curr];
    
    is_same = true;
    for i=1:6
        str_gui = get(handles.(tags_pos{i}), 'String');
        val_gui = str2double(str_gui);
        if abs(val_gui - vals_curr(i)) > 0.001
            is_same = false;
            break;
        end
    end
    
    if is_same
        q_sol = q_curr;
        success = true;
    end
    
    if ~success
        warning('off', 'Robotics:SerialLink:ikine');
        
        q0_guess = q_curr;
        if abs(q_curr(5)) < (1*pi/180) 
            q0_guess(5) = q0_guess(5) + 0.01;
        end
        
        try
            q_try = handles.robot.ikine(T_target, 'q0', q0_guess, 'mask', [1 1 1 1 1 1], 'tol', 1e-6);
            if verify_solution(handles.robot, q_try, T_target)
                q_sol = q_try;
                success = true;
            end
        catch
        end
        
        if ~success
            try
                T_pert = T_target * trotx(0.1 * pi/180);
                q_pert = handles.robot.ikine(T_pert, 'q0', q0_guess, 'mask', [1 1 1 1 1 1]);
                if ~isempty(q_pert)
                    q_try = handles.robot.ikine(T_target, 'q0', q_pert, 'mask', [1 1 1 1 1 1]);
                    if verify_solution(handles.robot, q_try, T_target)
                        q_sol = q_try;
                        success = true;
                    end
                end
            catch
            end
        end
        
        if ~success
            guess_list = [0 0 0 0 0 0; 0 -pi/4 pi/2 0 pi/4 0];
            for k=1:size(guess_list,1)
                try
                    q_try = handles.robot.ikine(T_target, 'q0', guess_list(k,:), 'mask', [1 1 1 1 1 1]);
                    if verify_solution(handles.robot, q_try, T_target)
                        q_sol = q_try;
                        success = true;
                        break;
                    end
                catch
                end
            end
        end
        warning('on', 'Robotics:SerialLink:ikine');
    end
    
    if success
        for i = 1:6
            diff = q_sol(i) - q_curr(i);
            k = round(diff / (2*pi));
            q_sol(i) = q_sol(i) - k * 2 * pi;
        end
        if ~is_same && abs(q_sol(5)) < (2 * pi/180)
            sum_46 = q_sol(4) + q_sol(6);
            q_opt = q_sol;
            q_opt(4) = q_curr(4);
            q_opt(6) = sum_46 - q_opt(4);
            
            if verify_solution(handles.robot, q_opt, T_target)
                q_sol = q_opt;
            end
        end
        for i=1:6
            set(handles.(sprintf('edit_j%d', i)), 'String', sprintf('%.3f', q_sol(i)*180/pi)); 
        end
        handles.robot.animate(q_sol);
        
        if ~is_same
            T_res = handles.robot.fkine(q_sol);
            err = norm(T_target(1:3,4) - T_res.t);
            fprintf('>> IK 计算完成. 误差: %.5f mm\n', err*1000);
        end
    else
        errordlg('IK 求解失败：目标不可达或奇异。');
    end
end

function isValid = verify_solution(robot, q, T_target)
    isValid = false;
    if isempty(q) || any(isnan(q)), return; end
    T_check = robot.fkine(q);
    if norm(T_target(1:3,4) - T_check.t) < 0.005 && norm(T_target(1:3,1:3) - T_check.R, 'fro') < 0.1
        isValid = true;
    end
end

% 显示工作空间
function btn_show_ws_Callback(hObject, eventdata, handles)
    % -----------------------------------------------------
    % 清理所有残留（轨迹、旧点云等）
    axes(handles.axes_3d); 
    cla(handles.axes_3d); % 清除残留图像


    view(45, 30);
    light('Position', [1, 1, -1], 'color', 'w'); % 打光，不然机械臂有点暗
    light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
    handles.robot.plot3d(zeros(1,6), ...
                         'workspace', handles.ws,  ...
                         'scale',0.6, ...
                         'path', handles.file_path, ...
                         'nowrist'); % 重绘机器人
    hold(handles.axes_3d, 'on'); 
    grid(handles.axes_3d, 'on');
    % -----------------------------------------------------

    num_points = 3000; % 采样点数
    q_lim = handles.qlim;
    
    % 随机生成关节角
    q_rand = q_lim(:,1)' + (q_lim(:,2)-q_lim(:,1))' .* rand(num_points, 6);
    
    points = zeros(num_points, 3);
    hWait = waitbar(0, '计算工作空间...');
    
    try
        for i = 1:num_points
            if mod(i, 200) == 0
                waitbar(i/num_points, hWait); 
            end
            T = handles.robot.fkine(q_rand(i,:));
            points(i,:) = T.t';
        end
        if ishandle(hWait)
            delete(hWait); 
        end
        
        % 绘制点云
        plot3(points(:,1), points(:,2), points(:,3), '.', 'Color', [0 0 1], 'MarkerSize', 1);
        title(handles.axes_3d, '机器人工作空间');
        
    catch ME
        if ishandle(hWait)
            delete(hWait); 
        end
        errordlg(ME.message);
    end
end

% =========================================================================
% 轨迹规划
% =========================================================================

% 读取当前位姿到起点
function btn_read_start_Callback(hObject, eventdata, handles)
    q = get_current_joints(handles);
    T = handles.robot.fkine(q);
    pos = T.t';
    for i=1:3
        tag = sprintf('edit_sx%d', i); 
        set(handles.(tag), 'String', sprintf('%.3f', pos(i))); 
    end
end

% 读取当前位姿到终点
function btn_read_end_Callback(hObject, eventdata, handles)
    q = get_current_joints(handles);
    T = handles.robot.fkine(q);
    pos = T.t';
    for i=1:3
        tag = sprintf('edit_ex%d', i); 
        set(handles.(tag), 'String', sprintf('%.3f', pos(i))); 
    end
end

% 运行单段轨迹
function btn_run_traj_Callback(hObject, eventdata, handles)
    % -----------------------------------------------------
    % 运行前清空3D视图残留和2D波形图
    axes(handles.axes_3d); 
    cla(handles.axes_3d);
    q_curr = get_current_joints(handles);
    view(45, 30);
%     light('Position', [1, 1, -1], 'color', 'w'); % 打光，不然机械臂有点暗
%     light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
    handles.robot.plot3d(q_curr, ...
                         'workspace', handles.ws,  ...
                         'scale',0.6, ...
                         'path', handles.file_path, ...
                         'nowrist'); % 重绘机器人
    hold(handles.axes_3d, 'on'); 
    grid(handles.axes_3d, 'on');
    
    % 清空波形图
    cla(handles.axes_q); 
    cla(handles.axes_qd); 
    cla(handles.axes_qdd);
    % -----------------------------------------------------

    t_total = str2double(get(handles.edit_time, 'String'));
    n_points = round(t_total * 30);
    
    % 读取 XYZ
    pos_s = zeros(1,3); 
    pos_e = zeros(1,3);
    for i=1:3
        pos_s(i) = str2double(get(handles.(sprintf('edit_sx%d',i)), 'String')); 
        pos_e(i) = str2double(get(handles.(sprintf('edit_ex%d',i)), 'String')); 
    end
    
    % 姿态设定
    default_rpy = [180 0 180] * pi/180; 
    
    % 读取速度/加速度边界
    v0 = parse_vector(get(handles.edit_sv, 'String'), 6);
    vf = parse_vector(get(handles.edit_ev, 'String'), 6);
    a0 = parse_vector(get(handles.edit_sa, 'String'), 6);
    af = parse_vector(get(handles.edit_ea, 'String'), 6);
    
    % 计算起点逆解
    T_s = transl(pos_s) * rpy2tr(default_rpy);
    try
        q_s_ik = handles.robot.ikine(T_s, 'q0', q_curr, 'mask', [1 1 1 1 1 1]);
    catch
        q_s_ik = my_ikine_epson(T_s, q_curr);
    end
    
    type_idx = get(handles.popup_type, 'Value');
    
    try
        switch type_idx
            case 1 % 三次
                [q,qd,qdd,t,pos] = ptp_trajectory(handles.robot, pos_s, pos_e, default_rpy, t_total, handles.qlim, n_points, q_s_ik, 'cubic', v0, vf);
            case 2 % 五次
                [q,qd,qdd,t,pos] = ptp_trajectory(handles.robot, pos_s, pos_e, default_rpy, t_total, handles.qlim, n_points, q_s_ik, 'quintic', v0, vf, a0, af);
            case 3 % S曲线
                [q,qd,qdd,t,pos] = ptp_trajectory(handles.robot, pos_s, pos_e, default_rpy, t_total, handles.qlim, n_points, q_s_ik, 's');
            case 4 % 直线
                [q,qd,qdd,t,pos] = line_trajectory(handles.robot, pos_s, pos_e, default_rpy, t_total, handles.qlim, n_points, q_s_ik);
            case 5 % 圆弧
                via = str2num(get(handles.edit_via, 'String')); %#ok<ST2NM>
                [q,qd,qdd,t,pos] = arc_trajectory(handles.robot, pos_s, via, pos_e, default_rpy, t_total, handles.qlim, n_points, q_s_ik);
        end
        
        plot_curves(handles, t, q, qd, qdd);
        
        axes(handles.axes_3d);
        plot3(pos(:,1), pos(:,2), pos(:,3), 'b-', 'LineWidth', 2);
        % handles.robot.animate(q);
        
        view(45, 30);
        light('Position', [0, 0, 0], 'color', 'w'); % 打光，不然机械臂有点暗
%         light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
        handles.robot.plot3d(q, ...
                             'workspace', handles.ws,  ...
                             'scale',0.6, ...
                             'path', handles.file_path, ...
                             'nowrist',...
                             'trail', {'r--', 'linewidth', 1},...
                             'fps', 20); % 重绘机器人
        hold(handles.axes_3d, 'on'); 
        grid(handles.axes_3d, 'on');
        
        % 更新当前关节显示
        for i=1:6
            set(handles.(sprintf('edit_j%d',i)), 'String', sprintf('%.3f', q(end,i)*180/pi)); 
        end
    catch ME
        errordlg(ME.message);
    end
end

% =========================================================================
% 全过程演示
% =========================================================================
function btn_run_main_Callback(hObject, eventdata, handles)
    path_points = {
        [0.4095, 0.0000, 0.5700]; [0.35, -0.10, 0.65]; [0.55, 0.15, 0.40];   
        [0.35, -0.15, 0.65];      [0.50, 0.15, 0.40]; [0.35, -0.20, 0.65];    
        [0.45, 0.15, 0.40];       [0.35, -0.10, 0.60];[0.55, 0.10, 0.40];     
        [0.35, -0.15, 0.60];      [0.50, 0.10, 0.40]; [0.35, -0.20, 0.60];    
        [0.45, 0.10, 0.40];
    };
    path_types = {'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP', 'PTP'};
    segment_times = [2.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0];
    
    % -----------------------------------------------------
    % 清除 3D 视图和 2D 视图
    q_init = zeros(1, 6);
    axes(handles.axes_3d); 
    cla(handles.axes_3d);
    view(45, 30);
%     light('Position', [1, 1, -1], 'color', 'w'); % 打光，不然机械臂有点暗
%     light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
    handles.robot.plot3d(q_init, ...
                         'workspace', handles.ws,  ...
                         'scale',0.6, ...
                         'path', handles.file_path, ...
                         'nowrist'); % 重绘机器人
    hold(handles.axes_3d, 'on'); 
    grid(handles.axes_3d, 'on');
    
    cla(handles.axes_q); cla(handles.axes_qd); cla(handles.axes_qdd);
    % -----------------------------------------------------
    
    for k=1:length(path_points)
        pt = path_points{k};
        plot3(pt(1), pt(2), pt(3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        text(pt(1), pt(2), pt(3)+0.02, sprintf('P%d', k), 'FontSize', 8);
    end
    
    hWait = waitbar(0, '正在计算全过程轨迹数据，请稍候...');
    
    all_q = []; all_qd = []; all_qdd = []; all_t = []; all_pos = [];
    current_time = 0;
    q_last = q_init;
    v0 = zeros(1,6); vf = zeros(1,6); a0 = zeros(1,6); af = zeros(1,6);
    default_rpy = [0 180 0] * pi/180;
    
    try
        for i = 1:length(path_types)
            waitbar(i/length(path_types), hWait);
            
            start_pt = path_points{i};
            end_pt = path_points{mod(i, length(path_points))+1};
            T_seg = segment_times(i);
            
            if strcmp(path_types{i}, 'PTP')
                [q, qd, qdd, t, pos] = ptp_trajectory(handles.robot, start_pt, end_pt, default_rpy, T_seg, handles.qlim, 50, q_last, 'quintic', v0, vf, a0, af);
            else
                [q, qd, qdd, t, pos] = line_trajectory(handles.robot, start_pt, end_pt, default_rpy, T_seg, handles.qlim, 50, q_last);
            end
            
            q_last = q(end,:);
            
            if isempty(pos)
                pos = zeros(size(q,1), 3);
                for k=1:size(q,1), tt=handles.robot.fkine(q(k,:)); pos(k,:)=tt.t'; end
            end
            if i==1
                all_q=q; all_qd=qd; all_qdd=qdd; all_t=t; all_pos=pos;
            else
                all_q = [all_q; q(2:end,:)]; 
                all_qd = [all_qd; qd(2:end,:)]; 
                all_qdd = [all_qdd; qdd(2:end,:)];
                all_t = [all_t; t(2:end) + current_time];
                all_pos = [all_pos; pos(2:end,:)];
            end
            current_time = all_t(end);
        end
        
        if ishandle(hWait), delete(hWait); end
        
        plot_curves(handles, all_t, all_q, all_qd, all_qdd);
        
%         % 替代plot_curves，为了保证qd和qdd的坐标轴不会过大
%         colors = lines(6);
%         legs = {'J1','J2','J3','J4','J5','J6'};
% 
%         axes(handles.axes_q); cla; hold on;
%         for j=1:6
%             plot(all_t, all_q(:,j), 'Color', colors(j,:)); 
%         end
%         axis tight; grid on;
%         legend(legs, 'Location', 'NorthEast', 'FontSize', 5, 'Box', 'off');
% 
%         axes(handles.axes_qd); cla; hold on;
%         for j=1:6
%             plot(all_t, all_qd(:,j), 'Color', colors(j,:)); 
%         end
%         axis([0 35 -1 1]); grid on;
%         legend(legs, 'Location', 'NorthEast', 'FontSize', 5, 'Box', 'off');
% 
%         axes(handles.axes_qdd); cla; hold on;
%         for j=1:6
%             plot(all_t, all_qdd(:,j), 'Color', colors(j,:)); 
%         end
%         axis([0 35 -1 1]); grid on;
%         legend(legs, 'Location', 'NorthEast', 'FontSize', 5, 'Box', 'off');
        
        axes(handles.axes_3d);
        plot3(all_pos(:,1), all_pos(:,2), all_pos(:,3), 'b-', 'LineWidth', 0.5);
        
        view(45, 30);
        light('Position', [0, 0, 0], 'color', 'w'); % 打光，不然机械臂有点暗
%         light('Position', [1, 1, 1], 'color', 'w'); % 打光，不然机械臂有点暗
        handles.robot.plot3d(all_q, ...
                             'workspace', handles.ws,  ...
                             'scale',0.6, ...
                             'path', handles.file_path, ...
                             'nowrist',...
                             'trail', {'r--', 'linewidth', 1},...
                             'fps', 20); % 重绘机器人
        hold(handles.axes_3d, 'on'); 
        grid(handles.axes_3d, 'on');
        
%         drawnow; 
%         
%         run_sync_animation(handles, all_t, all_q, 0);
%         
%         handles.robot.animate(all_q(end,:));
        
    catch ME
        if ishandle(hWait), delete(hWait); end
        errordlg(['演示出错: ' ME.message]);
    end
end

function q = get_current_joints(handles)
    q = zeros(1,6);
    for i=1:6
        q(i) = str2double(get(handles.(sprintf('edit_j%d', i)), 'String')) * pi/180; 
    end
end

function val = parse_vector(str, n)
    v = str2num(str); 
    if isempty(v)
        val = zeros(1,n); 
    elseif length(v)==1
        val = repmat(v,1,n); 
    else
        val = v; 
    end
end

function plot_curves(handles, t, q, qd, qdd)
    colors = lines(6);
    legs = {'J1','J2','J3','J4','J5','J6'};
    
    axes(handles.axes_q); cla; hold on;
    for j=1:6
        plot(t, q(:,j), 'Color', colors(j,:)); 
    end
    axis tight; grid on;
    legend(legs, 'Location', 'NorthEast', 'FontSize', 5, 'Box', 'off');
    
    axes(handles.axes_qd); cla; hold on;
    for j=1:6
        plot(t, qd(:,j), 'Color', colors(j,:)); 
    end
    axis tight; grid on;
    legend(legs, 'Location', 'NorthEast', 'FontSize', 5, 'Box', 'off');
    
    axes(handles.axes_qdd); cla; hold on;
    for j=1:6
        plot(t, qdd(:,j), 'Color', colors(j,:)); 
    end
    axis tight; grid on;
    legend(legs, 'Location', 'NorthEast', 'FontSize', 5, 'Box', 'off');
end

function popup_type_Callback(hObject, eventdata, handles)
    if get(hObject,'Value')==5
        set(handles.edit_via,'Enable','on'); 
    else
        set(handles.edit_via,'Enable','off'); 
    end
end

% =========================================================================
% 空回调函数
% =========================================================================
function edit_j1_Callback(hObject, eventdata, handles)
end
function edit_j1_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_j2_Callback(hObject, eventdata, handles)
end
function edit_j2_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_j3_Callback(hObject, eventdata, handles)
end
function edit_j3_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_j4_Callback(hObject, eventdata, handles)
end
function edit_j4_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_j5_Callback(hObject, eventdata, handles)
end
function edit_j5_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_j6_Callback(hObject, eventdata, handles)
end
function edit_j6_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_p1_Callback(hObject, eventdata, handles)
end
function edit_p1_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_p2_Callback(hObject, eventdata, handles)
end
function edit_p2_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_p3_Callback(hObject, eventdata, handles)
end
function edit_p3_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_p4_Callback(hObject, eventdata, handles)
end
function edit_p4_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_p5_Callback(hObject, eventdata, handles)
end
function edit_p5_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_p6_Callback(hObject, eventdata, handles)
end
function edit_p6_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_sx1_Callback(hObject, eventdata, handles)
end
function edit_sx1_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_sx2_Callback(hObject, eventdata, handles)
end
function edit_sx2_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_sx3_Callback(hObject, eventdata, handles)
end
function edit_sx3_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_ex1_Callback(hObject, eventdata, handles)
end
function edit_ex1_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_ex2_Callback(hObject, eventdata, handles)
end
function edit_ex2_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_ex3_Callback(hObject, eventdata, handles)
end
function edit_ex3_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_sv_Callback(hObject, eventdata, handles)
end
function edit_sv_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_sa_Callback(hObject, eventdata, handles)
end
function edit_sa_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_ev_Callback(hObject, eventdata, handles)
end
function edit_ev_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_ea_Callback(hObject, eventdata, handles)
end
function edit_ea_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_time_Callback(hObject, eventdata, handles)
end
function edit_time_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function edit_via_Callback(hObject, eventdata, handles)
end
function edit_via_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function popup_type_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function btn_show_ws_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
function run_sync_animation(handles, t, q, start_time_offset)
    if nargin < 4, start_time_offset = 0; end
    
    hAx = [handles.axes_q, handles.axes_qd, handles.axes_qdd];
    cursors = gobjects(1,3);
    
    for k = 1:3
        axes(hAx(k)); 
        hold on;
        cursors(k) = plot([start_time_offset, start_time_offset], ylim, 'r--', 'LineWidth', 1.5);
    end
    
    step = 2; 
    N = length(t);
    
    for i = 1:step:N
        handles.robot.animate(q(i,:));
        
        current_t = t(i) + start_time_offset;
        for k = 1:3
            set(cursors(k), 'XData', [current_t, current_t]);
        end
        drawnow limitrate; 
    end
    
    delete(cursors);
end