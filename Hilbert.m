%% 0. 检查数据源 (假设 u 存在)
function inst_freq_smooth=Hilbert(u,fs)
if ~exist('u', 'var')
    error('请先加载数据变量 u');
end

gyro_data = u(4:5,:)'; 

%% 2. 核心处理 Step 1: PCA 提取主振动轴
% -------------------------------------------------------------------------
% 为什么不用模值？因为模值会使频率翻倍。
% PCA 能自动找到振动最剧烈的那个方向，保留正弦波特性。
[coeff, score, ~] = pca(gyro_data); 
main_axis_data = score(:, 1); % 第一主成分 (振动最强的轴)

%% 3. 核心处理 Step 2: 滤波预处理 (至关重要)
% -------------------------------------------------------------------------
% 希尔伯特变换要求信号围绕0轴震荡，且必须去除高频毛刺
% 1. 去直流 (Highpass): 去除零偏和超低频漂移
fc_high = 0.5; % 截止频率 0.5Hz
data_hp = highpass(main_axis_data, fc_high, fs);

% 2. 去高频噪声 (Lowpass): 步态分析通常不超过 10Hz
fc_low =10;  % 截止频率 8Hz (根据需要调整)
data_clean = lowpass(data_hp, fc_low, fs);

% 3. 强力平滑 (可选): 进一步抹平微小毛刺
data_clean = smoothdata(data_clean, 'movmean', 5);

%% 4. 核心处理 Step 3: 希尔伯特变换提取频率与幅度
% -------------------------------------------------------------------------
z = hilbert(data_clean);

% A. 提取瞬时幅度 (Envelope) -> 用作"门控"
inst_amp = abs(z); 

% B. 提取瞬时相位 -> 求导得频率
inst_phase = unwrap(angle(z));
inst_freq_raw = diff(inst_phase) / (2*pi) * fs; 
inst_freq_raw = [inst_freq_raw; inst_freq_raw(end)]; % 补齐长度
inst_freq_raw = abs(inst_freq_raw); % 频率只有正值

%% 5. 核心处理 Step 4: 阈值门控与平滑
% -------------------------------------------------------------------------
% 策略：利用瞬时幅度判断"是否真的在动"
% 如果幅度很小(比如只有底噪)，则强制认为频率无效(归零或设为NaN)

% 【自动计算阈值】：取信号中位数的 3 倍作为底噪阈值 (自适应)
noise_floor = median(inst_amp); 
amp_threshold = max(0.2, noise_floor * 3); % 设定一个硬下限 0.2 (rad/s 或 deg/s)

% 应用阈值
final_freq = inst_freq_raw;
is_static = inst_amp < amp_threshold;
final_freq(is_static) = 0; % 或者设为 NaN 用于绘图时不显示

% 对频率进行滑动平均平滑 (去除求导带来的噪声)
inst_freq_smooth = smoothdata(final_freq, 'movmean', fs/5); % 0.2秒窗口

%% 6. 结果可视化
% -------------------------------------------------------------------------

end