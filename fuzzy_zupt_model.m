function K_zupt = fuzzy_zupt_model(acc_err, gyro_mag, freq)
% FUZZY_ZUPT_MODEL_FINAL 
% [Stance-Phase Optimized] 支撑相专用版
% 假设前提: 输入数据已经通过 GLRT，代表行走中的"疑似静止"片段。
% 设计目标: 90%的数据(正常支撑相) -> K=0.1; 10%的数据(GLRT漏网之鱼) -> K=1000

    %% 1. 输入限幅 (Clamping)
    % 保护机制：防止极个别离谱的异常值破坏计算
    acc_in  = min(4.0, abs(acc_err)); 
    gyro_in = min(2.0, abs(gyro_mag));
    freq_in = min(3.0, abs(freq));

    %% 2. 参数定义 (基于支撑相直方图的"包容性"设计)
    
    % --- Input 1: 加速度差 (Acc) ---
    % 直方图显示: 正常支撑相集中在 0~0.5
    % 策略: 0.6 以内全是满分静止(Z=1.0); 超过 2.5 才算不可接受的运动
    mf_acc.means  = [0.6,   1.5,   2.5]; 
    mf_acc.sigmas = [0.4,   0.5,   0];   
    
    % --- Input 2: 角速度 (Gyro) ---
    % 直方图显示: 正常支撑相有 0.15 左右的滚动
    % 策略: 0.25 以内全是满分静止(Z=1.0); 超过 1.0 才算显著晃动
    mf_gyro.means = [0.25,  0.6,   1.0]; 
    mf_gyro.sigmas = [0.2,   0.2,   0];   
    
    % --- Input 3: 频率 (Freq) ---
    % 直方图显示: 主要是 0, 噪声在 1.5
    % 策略: 0.5 以内全是满分静止; 1.5 附近才开始惩罚
    mf_freq.means = [0.5,   1.0,   1.8]; 
    mf_freq.sigmas = [0.3,   0.3,   0]; 
    
    % --- Output: K 值 (信任优先) ---
    % 既然 GLRT 已经筛选过一次，我们默认倾向于信任。
    % 只有当数据真的属于 B (Big) 时，才给 K7。
    % K_vals = [0.1, 0.5, 1, 5, 20, 100, 500];
    K_vals = [0.1, 0.5, 1, 10, 40, 100, 500];
    %         K1   K2   K3 K4 K5  K6   K7

    %% 3. 模糊化 (使用 Z/S 混合函数实现平顶)
    mu_a = calculate_hybrid_mfs(acc_in, mf_acc);
    mu_g = calculate_hybrid_mfs(gyro_in, mf_gyro);
    mu_f = calculate_hybrid_mfs(freq_in, mf_freq);

    %% 4. 推理 (逻辑优化：宽容模式)
    numerator = 0; denominator = 0;
    
    for i = 1:3 
        for j = 1:3 
            for k = 1:3 
                w = min([mu_a(i), mu_g(j), mu_f(k)]);
                
                if w > 0
                    % 获取最差指标的等级
                    max_lvl = max([i, j, k]);
                    
                    if max_lvl == 3      
                        % 只有检测到明确的 B (Level 3)，才启动拒绝模式
                        % 对应情况: 冲击极大(Acc>2.5) 或 摆动极大(Gyro>1.0)
                        k_idx = 7; % K=1000
                        
                    elseif max_lvl == 2  
                        % 如果只是 S (Level 2)，我们认为是"粗糙的静止"
                        % 这种情况下应该给弱修正，而不是拒绝
                        % 之前给 K4/K5，现在改为 K2/K3 (0.5 ~ 1.0)
                        k_idx = 3; % K=1.0 (适度信任)
                        
                    else                 
                        % 全是 Z (Level 1)
                        % 对应绝大多数正常的支撑相
                        k_idx = 1; % K=0.1 (完全信任)
                    end
                    
                    numerator = numerator + w * K_vals(k_idx);
                    denominator = denominator + w;
                end
            end
        end
    end

    if denominator == 0
        K_zupt = K_vals(7); 
    else
        K_zupt = numerator / denominator;
    end
end

%% =========================================================================
%  辅助函数: Z-Shape / S-Shape (平顶逻辑的核心)
% =========================================================================
function mus = calculate_hybrid_mfs(x, p)
    % p.means = [Z_limit, S_center, B_start]
    % p.sigmas = [Transition_Width, S_sigma, unused]
    
    mus = zeros(1,3);
    
    z_lim = p.means(1);     % 在这之前 Z=1.0
    trans_w = p.sigmas(1);  % 下降坡度
    
    s_cen = p.means(2);     % S 的中心
    s_sig = p.sigmas(2);    % S 的宽度
    
    b_start = p.means(3);   % 在这之后 B=1.0
    
    % --- 1. Zero (Z): 平顶 Z形函数 ---
    if x <= z_lim
        mus(1) = 1.0; % 【关键】只要小于阈值，满分信任
    elseif x <= z_lim + trans_w
        mus(1) = 1.0 - (x - z_lim) / trans_w;
    else
        mus(1) = 0;
    end
    
    % --- 3. Big (B): 饱和 S形函数 ---
    if x >= b_start
        mus(3) = 1.0;
    elseif x >= b_start - trans_w
        mus(3) = (x - (b_start - trans_w)) / trans_w;
    else
        mus(3) = 0;
    end
    
    % --- 2. Small (S): 高斯填充中间 ---
    mus(2) = exp(-(x - s_cen)^2 / (2 * s_sig^2));
    
    % 归一化
    sum_mu = sum(mus);
    if sum_mu > 0, mus = mus / sum_mu; end
end