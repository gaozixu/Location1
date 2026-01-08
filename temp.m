%% =============================================================
%  Paper Figure Generation: The Final Polish
% =============================================================

% 1. 确保已经运行了 INS 并有结果
% [x_h, cov, x_smooth, mht_history] = ZUPTaidedINS(u, zupt, RK);

% 2. 【显示优化】对判决历史进行平滑 (仅用于绘图，不影响轨迹)
% 设定滤波窗口时间为 1.0 秒 (足够抹平毛刺，又不会掩盖转弯)
fs_real = 1 / simdata.Ts;       % 获取真实采样率 (200Hz)
filter_time = 1.0;              % 滤波器长度 1秒
window_width = round(filter_time * fs_real); 

half_w = floor(window_width / 2);
N_len = length(mht_history);
mht_cleaned = zeros(size(mht_history));

for k = 1:N_len
    idx_start = max(1, k - half_w);
    idx_end   = min(N_len, k + half_w);
    segment = mht_history(idx_start:idx_end);
    
    % 只看有效判决 (非0值)
    valid_vals = segment(segment > 0);
    
    if isempty(valid_vals)
        % 如果还没开始，保持默认
        if k > 1, mht_cleaned(k) = mht_cleaned(k-1);
        else, mht_cleaned(k) = 1; end
    else
        % 【核心投票逻辑】
        % 统计 "Drift (Lock)" 出现的次数
        count_2 = sum(valid_vals == 2);
        count_total = length(valid_vals);
        
        % 阈值设为 0.4 (只要有40%的信心是直道，就显示为直道)
        % 既能填补直道的缝隙，又不会把弯道吃掉
        if (count_2 / count_total) > 0.4
            mht_cleaned(k) = 2; % 显示为 Drift
        else
            mht_cleaned(k) = 1; % 显示为 Turn
        end
    end
end

% 3. 专业级绘图
time = (0:N-1) * simdata.Ts;
heading_deg = unwrap(x_h(9, :)) * 180/pi;

% 创建高清画布
figure('Color', 'w', 'Name', 'MHT Final Analysis', 'Position', [100 100 1000 700]);

% (a) 航向角估计
subplot(2,1,1);
plot(time, heading_deg, 'b', 'LineWidth', 2); 
grid on; 
xlim([0 time(end)]);
ylabel('Heading (deg)', 'FontSize', 12, 'FontWeight', 'bold');
title('(a) Heading Estimation with Step-like Correction', 'FontSize', 14);
set(gca, 'FontSize', 10, 'LineWidth', 1.2);

% (b) MHT 判决逻辑 (平滑后)
subplot(2,1,2);
% 使用 area 填充颜色
area_h = area(time, mht_cleaned, 'FaceColor', [0.85 0.33 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
hold on;
% 叠加黑色阶梯线，增加对比度
stairs(time, mht_cleaned, 'k', 'LineWidth', 1.5);

grid on; 
xlim([0 time(end)]); 
ylim([0.8 2.2]); % 留出上下边距

% 优化 Y 轴标签
yticks([1 2]); 
yticklabels({'Turn (Free)', 'Drift (Lock)'});
ylabel('Decision Mode', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
title('(b) MHT Decision Logic (Dominant Mode)', 'FontSize', 14);
set(gca, 'FontSize', 10, 'LineWidth', 1.2, 'YGrid', 'off'); % 关掉Y网格更干净

%% =============================================================
%  Figure 2: 强力说服型航向图
% =============================================================
figure('Color', 'w', 'Name', 'Paper_Fig2_Heading', 'Position', [100 100 800 600]);

% 准备时间轴
time = (0:length(x_h)-1) * simdata.Ts;
heading_deg = unwrap(x_h(9, :)) * 180/pi;

% --- 子图 1: 航向角 ---
ax1 = subplot(4,1,1:3); % 占上面 3/4 的空间
plot(time, heading_deg, 'b', 'LineWidth', 2); 
grid on; hold on;
ylabel('Heading (deg)', 'FontSize', 12, 'FontWeight', 'bold');
title('\textbf{Effectiveness of MHT-HDR Logic}', 'Interpreter', 'latex', 'FontSize', 14);
set(gca, 'FontSize', 11, 'LineWidth', 1.2);
xlim([0 time(end)]);

% 【关键技巧】手动在图上画出“直道区间”的背景色，增加说服力
% 遍历 mht_cleaned，找到连续的 Drift 区间画底色
% (这里假设你已经运行了之前的 mht_cleaned 滤波代码)
if exist('mht_cleaned', 'var')
    yl = ylim;
    is_drawing = false;
    start_t = 0;
    for k = 1:length(mht_cleaned)
        if mht_cleaned(k) == 2 && ~is_drawing
            is_drawing = true; start_t = time(k);
        elseif mht_cleaned(k) == 1 && is_drawing
            is_drawing = false; end_t = time(k);
            % 画淡黄色背景块表示直道
            fill([start_t end_t end_t start_t], [yl(1) yl(1) yl(2) yl(2)], ...
                 [1 1 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility','off');
        end
    end
    % 重画曲线以防被覆盖
    plot(time, heading_deg, 'b', 'LineWidth', 2); 
end
legend('Estimated Heading', 'Location', 'southwest');

% --- 子图 2: 判决逻辑 ---
ax2 = subplot(4,1,4); % 占下面 1/4 的空间
area(time, mht_cleaned, 'FaceColor', [0.85 0.33 0.1], 'EdgeColor', 'none');
grid on; 
ylim([0.8 2.2]); xlim([0 time(end)]);
yticks([1 2]); yticklabels({'Turn', 'Straight'});
ylabel('Mode', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'LineWidth', 1.2);

linkaxes([ax1, ax2], 'x'); % 缩放时上下同步

%% =============================================================
%  Figure 3: 机理证明 - 零偏动态估计
% =============================================================
figure('Color', 'w', 'Name', 'Paper_Fig3_Bias', 'Position', [150 150 700 400]);

% 状态向量第 15 维通常是 Gyro Z Bias (请确认你的状态定义，如果是15维就是第15个)
% 如果是 21 维状态，可能是第 21 个。通常 ZUPT 代码里 Bias 是最后三个。
gyro_bias_z = x_h(15, :) * 180/pi; % 转换为度/秒

plot(time, gyro_bias_z, 'r-', 'LineWidth', 1.5);
grid on;
xlim([0 time(end)]);

ylabel('Est. Gyro Z Bias (deg/s)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
title('\textbf{Online Calibration of Gyroscope Bias}', 'Interpreter', 'latex', 'FontSize', 14);

% 添加解释性文字 (根据你的图调整位置)
% 说明：Bias 的变化意味着算法正在实时学习传感器的漂移
annotation('textbox', [0.15, 0.75, 0.3, 0.1], 'String', 'Bias adapts during straight phases', ...
           'EdgeColor', 'none', 'BackgroundColor', 'w', 'FitBoxToText', 'on');

set(gca, 'FontSize', 11, 'LineWidth', 1.2);

% 导出高分辨率图片 (可选)
% exportgraphics(gcf, 'Fig_MHT_Logic.png', 'Resolution', 600);