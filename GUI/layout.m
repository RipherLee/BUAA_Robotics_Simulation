function GenerateMyFig
    % GenerateMyFig - 界面生成脚本 (新增工作空间按钮)
    % 
    % 更新：在运动学面板底部增加了 [显示工作空间] 按钮。
    
    close all; clc;
    
    % --- 1. 创建主窗口 ---
    hFig = figure('Name', 'Epson Robot Control System', ...
        'NumberTitle', 'off', 'Position', [100, 100, 1200, 750], ...
        'MenuBar', 'none', 'ToolBar', 'figure', 'Color', [0.95 0.95 0.95], ...
        'Resize', 'on'); 

    % 辅助函数
    function h = create_ui(parent, style, str, pos, tag, fontsize, bg)
        if nargin < 6, fontsize = 10; end
        if nargin < 7, bg = [0.94 0.94 0.94]; end 
        if strcmp(style, 'pushbutton') || strcmp(style, 'togglebutton')
             bg = [0.96 0.96 0.96]; 
        end
        h = uicontrol(parent, 'Style', style, 'String', str, ...
            'Units', 'normalized', 'Position', pos, ...
            'FontSize', fontsize, 'Tag', tag, 'BackgroundColor', bg);
        if strcmp(style, 'edit'), set(h, 'BackgroundColor', [1 1 1]); end
        if strcmp(style, 'text'), set(h, 'HorizontalAlignment', 'left'); end
    end

    % --- 2. 顶部导航栏 ---
    hPanelNav = uipanel(hFig, 'Position', [0.01 0.92 0.28 0.07], 'Units', 'normalized', 'BorderType', 'none', 'BackgroundColor', [0.95 0.95 0.95]);
    create_ui(hPanelNav, 'togglebutton', '运动学',   [0.00 0.1 0.32 0.8], 'btn_nav_fk', 11, [0.9 0.9 0.9]);
    create_ui(hPanelNav, 'togglebutton', '轨迹规划', [0.34 0.1 0.32 0.8], 'btn_nav_traj', 11, [0.9 0.9 0.9]);
    create_ui(hPanelNav, 'togglebutton', '完整流程', [0.68 0.1 0.32 0.8], 'btn_nav_main', 11, [0.9 0.9 0.9]);

    % --- 3. 三个功能面板 (重叠布局) ---
    panel_pos = [0.01 0.35 0.28 0.56];
    hPanelFK   = uipanel(hFig, 'Title', '运动学', 'Units', 'normalized', 'Position', panel_pos, 'FontSize', 12, 'Tag', 'panel_fk', 'Visible', 'on');
    hPanelTraj = uipanel(hFig, 'Title', '轨迹规划', 'Units', 'normalized', 'Position', panel_pos, 'FontSize', 12, 'Tag', 'panel_traj', 'Visible', 'off'); 
    hPanelMain = uipanel(hFig, 'Title', '演示', 'Units', 'normalized', 'Position', panel_pos, 'FontSize', 12, 'Tag', 'panel_main', 'Visible', 'off');

    % --- 4. 绘图区 ---
    axes('Parent', hFig, 'Units', 'normalized', 'Position', [0.35 0.48 0.63 0.50], 'Tag', 'axes_3d');
    axes('Parent', hFig, 'Units', 'normalized', 'Position', [0.35 0.08 0.19 0.32], 'Tag', 'axes_q');
    axes('Parent', hFig, 'Units', 'normalized', 'Position', [0.57 0.08 0.19 0.32], 'Tag', 'axes_qd');
    axes('Parent', hFig, 'Units', 'normalized', 'Position', [0.79 0.08 0.19 0.32], 'Tag', 'axes_qdd');

    % --- 5. 运动学控件 (FK) ---
    create_ui(hPanelFK, 'text', '关节角度 (deg):', [0.05 0.92 0.4 0.05], '', 11);
    for i = 1:6
        y = 0.92 - i*0.07;
        create_ui(hPanelFK, 'text', sprintf('J%d:',i), [0.05 y 0.15 0.05], '', 10);
        create_ui(hPanelFK, 'edit', '0',               [0.20 y 0.20 0.05], sprintf('edit_j%d',i), 10);
    end
    
    create_ui(hPanelFK, 'text', '末端位姿 (XYZ RPY):', [0.50 0.92 0.45 0.05], '', 11);
    labels = {'X','Y','Z','R','P','Y'}; val_init = [0.4 0 0.5 180 0 180];
    for i = 1:6
        y = 0.92 - i*0.07;
        create_ui(hPanelFK, 'text', [labels{i} ':'],   [0.50 y 0.15 0.05], '', 10);
        create_ui(hPanelFK, 'edit', num2str(val_init(i)), [0.65 y 0.25 0.05], sprintf('edit_p%d',i), 10);
    end
    
    % 调整 FK/IK 按钮位置 (上移)
    create_ui(hPanelFK, 'pushbutton', '正解 FK', [0.1 0.20 0.35 0.08], 'btn_solve_fk', 11, [0.8 0.9 1]);
    create_ui(hPanelFK, 'pushbutton', '逆解 IK', [0.55 0.20 0.35 0.08], 'btn_solve_ik', 11, [0.8 1 0.8]);
    
    % [新增] 工作空间按钮
    create_ui(hPanelFK, 'pushbutton', '显示工作空间 (点云)', [0.1 0.08 0.80 0.08], 'btn_show_ws', 11, [1 0.9 1]);

    % --- 6. 轨迹规划控件 (Traj) ---
    % 起点
    create_ui(hPanelTraj, 'text', '起点 Start (XYZ):', [0.05 0.88 0.5 0.05], '', 11);
    xyz_start = [0.4, 0, 0.5]; labels_xyz = {'X','Y','Z'};
    for i=1:3
        create_ui(hPanelTraj, 'text', labels_xyz{i}, [0.05+(i-1)*0.3 0.82 0.1 0.05], '', 10);
        create_ui(hPanelTraj, 'edit', num2str(xyz_start(i)), [0.15+(i-1)*0.3 0.82 0.15 0.05], sprintf('edit_sx%d',i), 10);
    end
    % 起点 V/A
    create_ui(hPanelTraj, 'text', 'Vel:', [0.05 0.75 0.15 0.05], '', 9);
    create_ui(hPanelTraj, 'edit', '0',    [0.20 0.75 0.25 0.05], 'edit_sv', 9);
    create_ui(hPanelTraj, 'text', 'Acc:', [0.50 0.75 0.15 0.05], '', 9);
    create_ui(hPanelTraj, 'edit', '0',    [0.65 0.75 0.25 0.05], 'edit_sa', 9);
    create_ui(hPanelTraj, 'pushbutton', '读取当前(Start)', [0.05 0.68 0.4 0.06], 'btn_read_start', 9);

    % 终点
    create_ui(hPanelTraj, 'text', '终点 End (XYZ):', [0.05 0.58 0.5 0.05], '', 11);
    xyz_end = [0.4, -0.2, 0.4];
    for i=1:3
        create_ui(hPanelTraj, 'text', labels_xyz{i}, [0.05+(i-1)*0.3 0.52 0.1 0.05], '', 10);
        create_ui(hPanelTraj, 'edit', num2str(xyz_end(i)), [0.15+(i-1)*0.3 0.52 0.15 0.05], sprintf('edit_ex%d',i), 10);
    end
    % 终点 V/A
    create_ui(hPanelTraj, 'text', 'Vel:', [0.05 0.45 0.15 0.05], '', 9);
    create_ui(hPanelTraj, 'edit', '0',    [0.20 0.45 0.25 0.05], 'edit_ev', 9);
    create_ui(hPanelTraj, 'text', 'Acc:', [0.50 0.45 0.15 0.05], '', 9);
    create_ui(hPanelTraj, 'edit', '0',    [0.65 0.45 0.25 0.05], 'edit_ea', 9);
    create_ui(hPanelTraj, 'pushbutton', '读取当前(End)', [0.05 0.38 0.4 0.06], 'btn_read_end', 9);

    % 参数
    create_ui(hPanelTraj, 'text', 'Time(s):', [0.55 0.68 0.15 0.05], '', 10);
    create_ui(hPanelTraj, 'edit', '3.0',      [0.70 0.68 0.20 0.05], 'edit_time', 10);
    
    create_ui(hPanelTraj, 'text', 'Type:',    [0.55 0.38 0.15 0.05], '', 10);
    create_ui(hPanelTraj, 'popupmenu', {'关节-三次', '关节-五次', '关节-S曲线', '笛卡尔-直线', '笛卡尔-圆弧'}, ...
        [0.70 0.38 0.25 0.05], 'popup_type', 9);
    
    create_ui(hPanelTraj, 'text', '中间点(圆弧):', [0.05 0.28 0.3 0.05], '', 10);
    create_ui(hPanelTraj, 'edit', '0.45 -0.1 0.5', [0.35 0.28 0.55 0.05], 'edit_via', 10);
    
    create_ui(hPanelTraj, 'pushbutton', '生成轨迹', [0.1 0.1 0.8 0.12], 'btn_run_traj', 12, [1 0.9 0.8]);

    % --- 7. Main 演示 ---
    create_ui(hPanelMain, 'text', '执行 main.m 中的多段轨迹', [0.1 0.7 0.8 0.1], '', 12);
    create_ui(hPanelMain, 'pushbutton', '全过程演示', [0.1 0.4 0.8 0.2], 'btn_run_main', 14, [0.6 1 0.6]);

    disp('>>> 界面布局已更新！请在弹出的窗口中 File -> Save As -> GUI.fig 覆盖原文件');
end