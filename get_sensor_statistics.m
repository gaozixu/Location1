function [ sigma2_acc] = get_sensor_statistics(imu_window, sigma2_a, sigma2_g, g_val)
    % 计算 GLRT 和 局部加速度方差
    % 输入: 
    %   imu_window: Nx6 矩阵 (前3列加速度，后3列陀螺仪)
    %   sigma2_a, sigma2_g: 传感器噪声方差
    %   g_val: 重力加速度模长 (通常 9.81)
    
    [~] = size(imu_window);
    acc = imu_window(:, 1:3);

    
    % --- 2. 计算局部方差 (用于模糊输入) ---
    % 计算加速度模长的方差，或三轴方差之和
    % 这里使用三轴方差迹 (trace of covariance)
    sigma2_acc = trace(cov(acc));
end