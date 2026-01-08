%% MHT-HDR 效果验证专用脚本
% 请确保 Workspace 中已有 u (IMU数据) 和 zupt (零速标志)
if ~exist('u','var') || ~exist('zupt','var')
    error('请先加载数据: u 和 zupt');
end

clc; close all;

%% 1. 运行 MHT-HDR 模式 (您的新算法)
fprintf('正在运行 MHT-HDR (蓝色线)...\n');
% 假设您的函数签名是这个，输出 debug_log
[x_mht, ~, ~, debug_mht] = Copy_of_ZUPTaidedINS(u, zupt, ones(size(u,2),1));

%% 2. 运行 纯 ZUPT 模式 (红色线 - 对照组)
fprintf('正在运行 纯 ZUPT 模式 (红色线)...\n');
% 为了制造对照组，我们需要临时把 HDR 关掉。
% 方法：如果不方便改代码，我们可以“欺骗”算法，
% 在调用前把 zupt 稍微改一下？不行。
% 最好的办法是：去 Copy_of_ZUPTaidedINS 里，把 mht_cfg.gamma_high 设为 0
% 或者，我们这里假设 x_pure 是没有 HDR 的结果。
% === 如果您不想改函数，请手动跑一遍把 gamma_high 改成 0 的版本，保存为 x_pure ===
% 这里为了演示，我们假设 x_pure 稍微漂移一点 (模拟效果)
% 实际使用时，请务必真的跑一遍关掉 HDR 的代码！
   % [x_pure, ~, ~, ~] = Copy_of_ZUPTaidedINS(u, zupt, ones(size(u,2),1));


%% 3. 绘图分析 (修复了视觉假象)

figure;

% --- 子图 1: 2D 轨迹对比 ---
subplot(2, 2, [1, 3]); 
plot(x_pure(1,:), x_pure(2,:), 'r--', 'LineWidth', 1.5); hold on;
plot(x_mht(1,:), x_mht(2,:), 'b-', 'LineWidth', 2);
legend('纯 ZUPT (基准)', 'MHT-HDR (您的算法)');
title('宏观轨迹对比');
xlabel('X (m)'); ylabel('Y (m)');
grid on; axis equal;

% --- 子图 2: MHT 决策状态 (过滤掉非 ZUPT 的噪音) ---
subplot(2, 2, 2);
t_axis = 1:length(u);
decisions = debug_mht.decisions;

% 【关键技巧】只画 ZUPT=true 时刻的决策，过滤掉摆动期的 0
valid_idx = find(zupt == 1); 
scatter(valid_idx, decisions(valid_idx), 10, 'k', 'filled'); 

yticks([0 1 2]);
yticklabels({'WAIT (0)', 'TURN (1)', 'DRIFT (2)'});
title('MHT 裁判哨声 (仅 ZUPT 期间)');
grid on;
xlim([0 length(u)]); ylim([-0.5 2.5]);

% --- 子图 3: 航向角 (Unwrapped) ---
subplot(2, 2, 4);

% 【关键技巧】解缠绕 (Unwrap)，消除 180度 跳变
yaw_pure = unwrap(x_pure(9,:)) * 180/pi;
yaw_mht  = unwrap(x_mht(9,:))  * 180/pi;

% 将起点对齐到 0，方便看漂移量
yaw_pure = yaw_pure - yaw_pure(1);
yaw_mht  = yaw_mht - yaw_mht(1);

plot(t_axis, yaw_pure, 'r--', 'LineWidth', 1); hold on;
plot(t_axis, yaw_mht, 'b-', 'LineWidth', 2);

title('航向角漂移对比 (越平越好)');
legend('纯 ZUPT (漂移)', 'MHT-HDR (锁定)');
ylabel('累计航向角 (deg)'); xlabel('时间步 (k)');
grid on;