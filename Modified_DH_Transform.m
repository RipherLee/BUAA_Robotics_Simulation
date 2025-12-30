function T = Modified_DH_Transform(theta, d, a, alpha)
    % Modified_DH_Transform 自编程序：计算改进DH参数(Modified DH)的齐次变换矩阵
    % 改进DH参数即为前置DH参数，具体公式参考《机器人学基础》
    % 输入:
    %   theta{i} : 关节角 (rad)
    %   d{i}     : 偏置 (m)
    %   a{i-1}     : 杆长 (m)
    %   alpha{i-1} : 扭转角 (rad)
    % 输出:
    %   T{i-1}{i}     : 4x4 齐次变换矩阵 (double)

    % 改进 DH 变换顺序: Rot_x(alpha{i-1}) * Trans_x(a_{i-1}) * Rot_x(theta{i}) * Trans_z(d{i})
    
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    T = [ ct      -st      0      a;
          st*ca   ct*ca   -sa   -d*sa;
          st*sa   ct*sa    ca    d*ca;
           0       0       0      1 ];
end