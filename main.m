    
clc;clear ;close all;
global simdata;
%% Loads the algorithm settings and the IMU data
% x_h1= load("shoe_x_h.mat").x_h;
disp('Loads the algorithm settings and the IMU data')
u=setting();
N=length(u);
t=0:simdata.Ts:(N-1)*simdata.Ts;
fs = 1 / simdata.Ts;

% u=filiter_data(u);


%% Run the zero-velocity detector 
disp('Runs the zero velocity detector')
% 计算了zupt，glrt值T，加速度方差，角速度方差
[zupt, T, sigma2_a_k,sigma2_g_k]=zero_velocity_detector(u);

plot(zupt.*simdata.gamma);hold on;plot(T)

% 计算每个时刻与静止的偏离度，加速度模值为g，角速度模值为0
acc_norm_difference = vecnorm(u(1:3,:),2,1) - simdata.g;
gyro_norm_difference = vecnorm(u(4:6,:),2,1) ;
hold on;
plot(t,gyro_norm_difference,'b');
plot(t,acc_norm_difference,'r');
plot(t,zupt,'g');
inst_freq_smooth = Hilbert(u,fs);
% 把zupt为1所对应的模糊输入提取出来其他的都为0，画图看看效果
for i=1:N
    if zupt(i)==0
        acc_norm_difference(i) = 0;
        gyro_norm_difference(i) = 0 ;
        inst_freq_smooth(i) = 0;
    end
end
idx_zupt1 = (zupt == 1);  % 生成逻辑索引：zput=1的位置为true，其余为false
 pure_acc_norm_difference= acc_norm_difference(idx_zupt1); % 仅保留逻辑索引为true的元素
 pure_gyro_norm_difference= gyro_norm_difference(idx_zupt1); % 
 pure_inst_freq_smooth= inst_freq_smooth(idx_zupt1); % 
 
 % 绘制
%  plot( pure_inst_freq_smooth);
 % h1 = histogram(abs(pure_acc_norm_difference), 'BinEdges', 0:0.05:5, 'Normalization', 'probability');
 % h2 = histogram(abs(pure_gyro_norm_difference), 'BinEdges', 0:0.01:10, 'Normalization', 'probability');
 % h3 = histogram(pure_inst_freq_smooth, 'BinEdges', 0:0.1:5, 'Normalization', 'probability');


inst_freq_smooth(1:2500,1)=0;
 inst_freq_smooth(32100:end,1)=0;
plot(zupt);hold on;plot(inst_freq_smooth);

figure; 
subplot(3,1,1); plot(pure_acc_norm_difference); title('静止段 Acc Err');
subplot(3,1,2); plot(pure_gyro_norm_difference); title('静止段 Gyro Mag');
subplot(3,1,3); plot(pure_inst_freq_smooth); title('静止段 Freq');

% K_scale=zeros(1,17110);
% for i = 1:17110
%     K_scale(i) = fuzzy_zupt_model(abs(pure_acc_norm_difference(i)),pure_gyro_norm_difference(i),inst_freq_smooth(i));
% end
% plot(K_scale)
% h3 = histogram(K_scale, 'BinEdges', 0:01:500, 'Normalization', 'probability');
RK = zeros(N,1);
for i = 1:N
    RK(i) = fuzzy_zupt_model(abs(acc_norm_difference(i)),gyro_norm_difference(i),inst_freq_smooth(i));
    % RK(i) =1;
end
plot(log(RK));
axis([19000 20000 -2 6])
%% Run the Kalman filter
disp('Runs the filter')
[x_h, cov, x_smooth , mht_history] = ZUPTaidedINS(u, zupt, RK);


%% 4. 绘图对比 (这部分直接用于论文)
% 提取位置 (假设 1:3 是 x, y, z)
pos_zupt = x_h(1:3, :);
pos_rts  = x_smooth(1:3, :);

% --- 图1: 2D 平面轨迹对比 ---
fig=figure('Color', 'none', 'InvertHardcopy', 'off', 'Name', '2D Trajectory');
ax = axes(fig);
hold on; axis equal;
% 画起点
plot(0, 0, 'ks', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
% plot(x_h1(1,:), x_h1(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'ZUPT');
% 画 fuzzy-ZUPT 轨迹
plot(pos_zupt(1,:), pos_zupt(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Fuzzy Adaptive ZUPT');
% 画 RTS 轨迹 (重点！)
plot(pos_rts(1,:), pos_rts(2,:), 'r-.', 'LineWidth', 2, 'DisplayName', 'Proposed (Fuzzy + RTS)');
box on;
xlabel('East (m)');
ylabel('North (m)');
legend('show', 'Location', 'best');
title('Trajectory Comparison: Effectiveness of RTS Smoothing');
set(gca, 'Color', 'none', 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');


% --- 图2: 高度 (Z轴) 对比 ---
% 高度通常是纯惯导漂移最严重的，RTS 修正效果通常非常震撼
figure('Color', 'w', 'Name', 'Height Comparison');
hold on; grid on;
time_axis = (1:N) * simdata.Ts;
plot(time_axis, pos_zupt(3,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Fuzzy Adaptive ZUPT');
plot(time_axis, pos_rts(3,:), 'r-', 'LineWidth', 2, 'DisplayName', 'Proposed (Fuzzy + RTS)');
xlabel('Time (s)');
ylabel('Height (m)');
legend('show');
title('Height Drift Correction');

%% 5. 计算闭合误差 (论文中的定量分析)
% 假设你是闭环行走（回到了原点）
start_pos = [0; 0; 0]; % 或者 x_smooth(:,1)
end_pos_zupt = pos_zupt(:, end);
end_pos_rts  = pos_rts(:, end);

error_zupt = norm(end_pos_zupt(1:2) - start_pos(1:2)); % 2D 误差
error_rts  = norm(end_pos_rts(1:2) - start_pos(1:2));  % 2D 误差

fprintf('========================================\n');
fprintf('闭合误差对比 (Loop Closure Error):\n');
fprintf('1. Fuzzy ZUPT Only:  %.4f meters\n', error_zupt);
fprintf('2. Fuzzy ZUPT + RTS: %.4f meters\n', error_rts);
fprintf('----------------------------------------\n');
fprintf('精度提升 (Improvement): %.2f%%\n', (error_zupt - error_rts)/error_zupt * 100);
fprintf('========================================\n');

%% Print the horizontal error and spherical error at the end of the
%% trajectory
% sprintf('Horizontal error = %0.5g , Spherical error = %0.5g',sqrt(sum((x_h(1:2,end)).^2)), sqrt(sum((x_h(1:3,end)).^2)))
% tbl = load("new_data\processed_data22.mat");
% trace = tbl.gt;
%  figure('Color', 'none', 'InvertHardcopy', 'off');plot(trace(:,1),trace(:,2));
  set(gca, 'Color', 'none', 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none'); grid off;box on

%  figure('Color', 'none');
%  plot(x_h(2,:),x_h(1,:));hold;
%  set(gca, 'Color', 'none', 'XColor', 'none', 'YColor', 'none');
%  axis equal
% grid on
% box on
% exportgraphics(gcf, [filename '.png'], 'BackgroundColor', 'none', 'ContentType', 'image');
figure;
plot(unwrap(x_h(9,:)));

%% View the result 
% disp('Views the data')
% view_data;



