%% 滤波
function u = liliter_data(u)
global simdata;
%% Loads the algorithm settings and the IMU data
disp('Loads the algorithm settings and the IMU data')
u=setting();

acc = u(1:3,:);
gyro = u(4:6,:);

acc_norm = vecnorm(acc,2,1);
gyro_norm = vecnorm(gyro,2,1);

acc_norm = acc_norm - mean(acc_norm );
gyro_norm = gyro_norm - mean(gyro_norm);

fs = 100; L = length(u);
t = 0:1/fs:(L-1)/fs;
f = fs*(0:(L/2))/L;

% FFT - 加速度
Y_acc = fft(acc_norm);
P2_acc = abs(Y_acc/L);
P1_acc = P2_acc(1:floor(L/2)+1);
P1_acc(2:end-1) = 2*P1_acc(2:end-1);

% FFT - 角速度
Y_gyro = fft(gyro_norm);
P2_gyro = abs(Y_gyro/L);
P1_gyro = P2_gyro(1:floor(L/2)+1);
P1_gyro(2:end-1) = 2*P1_gyro(2:end-1);


fs = 100;            % 假设采样率是 100Hz (请根据实际情况修改)
fc = 30;             % 截止频率 10Hz (根据频谱图，保留了主要能量)
order = 2;           % 滤波器阶数 (双向滤波后等效为 4 阶，足够陡峭)

% 设计巴特沃斯低通滤波器
[b, a] = butter(order, fc/(fs/2), 'low');

%% 执行滤波 (针对三轴数据)
% 假设 raw_acc 和 raw_gyro 是你的原始数据矩阵，大小为 [N x 3]

% 1. 使用 filtfilt 进行零相移滤波 (关键！)
acc_filtered = zeros(size(acc));
gyro_filtered = zeros(size(gyro));

for i = 1:3
    acc_filtered(i, :) = filtfilt(b, a, acc(i, :));
    gyro_filtered(i, :) = filtfilt(b, a, gyro(i, :));
end
u = [acc_filtered;gyro_filtered];


%% 4. 绘图对比
% figure('Color', 'w', 'Position', [100, 100, 800, 600]);
% 
% % 上图：加速度 FFT
% subplot(2,1,1);
% plot(f, P1_acc, 'b', 'LineWidth', 1.5);
% title('加速度模值 FFT (Acceleration Norm)');
% xlabel('频率 (Hz)'); ylabel('幅值');
% xlim([0 8]); grid on;
% legend('Accel Spectrum');
% 
% % 下图：角速度 FFT
% subplot(2,1,2);
% plot(f, P1_gyro, 'r', 'LineWidth', 1.5);
% title('角速度模值 FFT (Gyroscope Norm)');
% xlabel('频率 (Hz)'); ylabel('幅值 (rad/s)');
% xlim([0 8]); grid on;
% legend('Gyro Spectrum');
% 
% figure('Color', 'w');
% plot(t, acc(1,:), 'Color', [0.8, 0.8, 0.8], 'DisplayName', '原始信号');
% hold on;
% plot(t, acc_filtered(1,:), 'b', 'LineWidth', 1.5, 'DisplayName', '10Hz 低通滤波后');
% title('原始数据 vs 滤波数据 (Acc X)');
% xlabel('时间 (s)'); ylabel('加速度 (m/s^2)');
% legend;
% grid on;
end
