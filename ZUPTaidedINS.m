function  [x_h, cov, x_smooth, mht_history]=ZUPTaidedINS(u,zupt,RK)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      Initialize the data fusion          %%       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N=length(u);
[P, Q, R_base, H]=init_filter; 
[x_h, cov, Id]=init_vec(N,P);
[x_h(1:9,1), quat]=init_Nav_eq(u);

% RTS 存储预分配
dim = size(x_h, 1);       
x_pred_store = zeros(dim, N);      
P_pred_store = zeros(dim, dim, N); 
P_post_store = zeros(dim, dim, N); 
F_store      = zeros(dim, dim, N); 
mht_history = zeros(1, N);


P_post_store(:,:,1) = P; 
x_pred_store(:,1) = x_h(:,1); 

count  = 0;

% ============================================================
% 【新增】 MHT-HDR 模块初始化参数
% ============================================================
mht_cfg.window_size = 50; 

% 【关键点】5度。
% 小于 5度 -> 强制锁死 (解决直道抖动)
% 大于 5度 -> 允许进入 PK (操场转弯通常 >10度，能被放进来)
mht_cfg.gamma_low   = 5.0 * (pi/180);  % 回归 5 度
mht_cfg.gamma_high  = 25.0 * (pi/180); 
mht_cfg.lambda_acc  = 2.0;             % 侧向力权重
% base_penalty 可以设为 0 或者很小 (比如 0.1)，因为数据干净了，不需要重罚
% 保持之前的物理权重
mht_cfg.lambda_bias = 20.0;

% MHT 运行时状态结构体
mht_state.active = false;      % 是否正在进行多假设分支
mht_state.timer  = 0;
mht_state.buffer = [];         % 缓冲区
mht_state.stable_bias = 0;     % 记忆的稳定 Bias
mht_state.cost_drift = 0;
mht_state.cost_turn  = 0;

% HDR 观测矩阵 (Heading Error 在状态向量的第9位)
H_hdr = zeros(1, dim); 
H_hdr(15) = 1; 
R_hdr = (0.5 * pi/180)^2; % HDR 观测噪声 (0.5度)
% ============================================================

% ============================================================
% 【新增】自动计算安装误差 (Mounting Bias Auto-Calibration)
% ============================================================
% 假设前 200 个样本 (1秒) 是静止的。
% 此时测得的 acc_y 完全由"安装歪了"导致。
init_samples = 200; 
if N > init_samples
    % 注意：这里用 u(2,:) 是原始加速度计Y轴数据
    mounting_bias_ay = mean(u(2, 1:init_samples));
else
    mounting_bias_ay = 0;
end

fprintf('自动校准完成: Y轴安装误差 = %.4f m/s^2\n', mounting_bias_ay);
% ============================================================

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Run the filter algorithm          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=2:N
    
    % 1. 传感器误差补偿
    u_h=comp_imu_errors(u(:,k),x_h(:,k-1)); 
    
    % 2. 导航方程更新
    [x_h(:,k), quat]=Navigation_equations(x_h(:,k-1),u_h,quat); 
    
    % 3. 状态转移矩阵 F
    [F, G]=state_matrix(quat,u_h); 
    
    % 4. 预测协方差 P
    P = F*P*F' + G*Q*G';
    P = (P+P')/2;
    
    % 存储预测数据
    x_pred_store(:, k)      = x_h(:,k);   
    P_pred_store(:, :, k)   = P;          
    F_store(:, :, k-1)      = F;          
    
    % -------------------------------------------------------------------
    % Step 5: Zero Velocity Update (ZUPT)
    % -------------------------------------------------------------------
    if zupt(k)==true
        count = count +1;
        current_R = diag([0.01 0.01 0.01].^2) .* 10 .* RK(k);
        K = (P*H')/(H*P*H' + current_R);
        z = -x_h(4:6,k);   
        dx = K*z;
        [x_h(:,k), quat] = comp_internal_states(x_h(:,k), dx, quat);
        P = (Id - K*H)*P;
        P = (P+P')/2;
    end
    
    % -------------------------------------------------------------------
    % Step 6: MHT-HDR (基于多假设的漂移抑制)
    % -------------------------------------------------------------------
    % 准备数据: 需要 Body 系下的 Gyro Z, Acc Y, 和 前向速度
    
    % A. 获取 Body 系 Gyro Z (u_h 已经去除了 Bias)
    % 注意: u 通常是 [acc; gyro] 6x1. 所以 gyro z 是 u(6)
   % 【核心修改】只有在 ZUPT 有效时，才运行 MHT 判决
    if zupt(k) == true
        % A. 获取数据
       if zupt(k) == true
        % A. 获取数据
        curr_w_z = u_h(6); 
        
        % 【核心修改】减去自动算出来的安装误差！
        % 这样 curr_acc_y 就变成了纯净的运动加速度，直道上近似为 0
        curr_acc_y = u_h(2) - mounting_bias_ay; 
        
        Rb2n = q2dcm(quat); 
        vel_n = x_h(4:6, k);
        vel_b = Rb2n' * vel_n; 
        curr_vel_fwd = vel_b(1); 
        
        % D. 运行 MHT 逻辑 (参数用回 5度)
        [mht_decision, mht_state] = run_mht_logic(curr_w_z, curr_acc_y, curr_vel_fwd, mht_state, mht_cfg);
        
        % 保存决策到历史记录
        mht_history(k) = mht_decision;
        
        % E. 执行 HDR 更新 (仅在判定为 Drift 时)
        if mht_decision == 2 
            z_hdr = 0 + u_h(6)*0.12;
            K_hdr = (P * H_hdr') / (H_hdr * P * H_hdr' + R_hdr);
            dx_hdr = K_hdr * z_hdr;
            [x_h(:,k), quat] = comp_internal_states(x_h(:,k), dx_hdr, quat);
            P = (Id - K_hdr * H_hdr) * P;
            P = (P+P')/2;
        end
        
    else
        % 【关键】如果是摆动相 (Swing Phase)，不运行逻辑，保持上一时刻的状态！
        % 这样图表就不会因为迈步而跳变了
        if k > 1
            mht_history(k) = mht_history(k-1); 
        else
            mht_history(k) = 1; % 默认
        end
        
    end
   end  
    % -------------------------------------------------------------------
    
    P_post_store(:, :, k) = P;
    cov(:,k)=diag(P);
    x_h(3,k)=0; % 高度通道简单锁定 (Barometer logic normally)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%       RTS Smoother (Equality Constrained)%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Running RTS Smoother with Equality Constraint Projection...\n');

x_smooth = x_h;             
P_smooth = P_post_store;    

% (A) 提取最后时刻
x_end_prior = x_h(:, end);
P_end_prior = P_post_store(:, :, end);

% (B) 定义几何约束: Dx = d (回到原点)
state_dim = size(x_h, 1);
D = zeros(3, state_dim); 
D(1:3, 1:3) = eye(3); 
d = [0; 0; 0]; 

% (C) 计算投影
constraint_error = D * x_end_prior - d;
Denominator = D * P_end_prior * D';
J = (P_end_prior * D') / Denominator; 

x_end_projected = x_end_prior - J * constraint_error;
I_mat = eye(state_dim);
P_end_projected = (I_mat - J * D) * P_end_prior;
P_end_projected = (P_end_projected + P_end_projected') / 2;

x_smooth(:, end) = x_end_projected;       
P_smooth(:, :, end) = P_end_projected;

% 反向平滑循环
for k = (N-1):-1:1
    x_curr_post = x_h(:, k);                
    x_next_pred = x_pred_store(:, k+1);     
    P_curr_post = P_post_store(:, :, k);    
    P_next_pred = P_pred_store(:, :, k+1);  
    F_curr      = F_store(:, :, k);         
    
    x_next_smooth = x_smooth(:, k+1);       
    Ck = P_curr_post * F_curr' / P_next_pred;
    
    x_smooth(:, k) = x_curr_post + Ck * (x_next_smooth - x_next_pred);
end
fprintf('RTS Smoothing Done.\n');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SUBFUNCTIONS (原有的辅助函数)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_h cov Id]=init_vec(N,P)
global simdata
if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    cov=zeros(21,N); x_h=zeros(21,N);
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    cov=zeros(15,N); x_h=zeros(15,N);
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    cov=zeros(15,N); x_h=zeros(15,N);
else
    cov=zeros(9,N); x_h=zeros(9,N);
end
Id=eye(size(P));
cov(:,1)=diag(P);
end

function [x quat]=init_Nav_eq(u)
global simdata;
f_u=mean(u(1,1:20));
f_v=mean(u(2,1:20));
f_w=mean(u(3,1:20));
roll=atan2(-f_v,-f_w);
pitch=atan2(f_u,sqrt(f_v^2+f_w^2));
attitude=[roll pitch simdata.init_heading]';
Rb2t=Rt2b(attitude)';
quat=dcm2q(Rb2t);
x=zeros(9,1);
x(1:3,1)=simdata.init_pos;
x(7:9,1)=attitude;
end

function [P Q R H]=init_filter
global simdata;
% 简化逻辑：只展示最常用的 15 状态 (Pos, Vel, Att, AccBias, GyroBias)
% 如果您的 simdata 配置不同，请保留原来的 init_filter
if (strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')) 
    P=zeros(15); Q=zeros(12); H=zeros(3,15);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_bias.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_bias.^2);
    Q(7:9,7:9)=diag(simdata.acc_bias_driving_noise.^2);
    Q(10:12,10:12)=diag(simdata.gyro_bias_driving_noise.^2);
else 
    % Fallback for other configs if needed, utilizing user's original logic
    P=zeros(15); Q=zeros(12); H=zeros(3,15); % Placeholder
end
% 通用部分
H(1:3,4:6)=eye(3); % ZUPT 观测速度
P(1:3,1:3)=diag(simdata.sigma_initial_pos.^2);
P(4:6,4:6)=diag(simdata.sigma_initial_vel.^2);
P(7:9,7:9)=diag(simdata.sigma_initial_att.^2);
Q(1:3,1:3)=diag(simdata.sigma_acc.^2);
Q(4:6,4:6)=diag(simdata.sigma_gyro.^2);
R=diag(simdata.sigma_vel.^2);
end

function [y,q]=Navigation_equations(x,u,q)
global simdata;
y=zeros(size(x));
Ts=simdata.Ts;
persistent old_w_tb
if isempty(old_w_tb), old_w_tb = zeros(3,1); end
w_curr = u(4:6);
theta_k   = w_curr * Ts;    
theta_km1 = old_w_tb * Ts;  
coning_correction = (1/12) * cross(theta_km1, theta_k);
phi_vec = theta_k + coning_correction;
P_ = phi_vec(1); Q_ = phi_vec(2); R_ = phi_vec(3);
OMEGA = 0.5 * [0 R_ -Q_ P_; -R_ 0 P_ Q_; Q_ -P_ 0 R_; -P_ -Q_ -R_ 0];
v = norm(phi_vec);
if v ~= 0
    q = (cos(v/2)*eye(4) + (2/v)*sin(v/2)*OMEGA) * q;
    q = q ./ norm(q); 
end
old_w_tb = w_curr;
Rb2t=q2dcm(q);
y(7)=atan2(Rb2t(3,2),Rb2t(3,3));
y(8)=-atan(Rb2t(3,1)/sqrt(1-Rb2t(3,1)^2));
y(9)=atan2(Rb2t(2,1),Rb2t(1,1));
g_t=[0 0 simdata.g]';
f_t=q2dcm(q)*u(1:3);
acc_t=f_t+g_t;
A=eye(6); A(1,4)=Ts; A(2,5)=Ts; A(3,6)=Ts;
B=[(Ts^2)/2*eye(3);Ts*eye(3)];
y(1:6)=A*x(1:6)+B*acc_t;
end

function [F, G]=state_matrix(q,u)
global simdata
Rb2t=q2dcm(q);
f_t=Rb2t*u(1:3);
St=[0 -f_t(3) f_t(2); f_t(3) 0 -f_t(1); -f_t(2) f_t(1) 0];
O=zeros(3); I=eye(3);
Da=diag(u(1:3)); Dg=diag(u(4:6));
B1=-1/simdata.acc_bias_instability_time_constant_filter*eye(3);
B2=-1/simdata.gyro_bias_instability_time_constant_filter*eye(3);
% 这里仅保留最常用的 15 状态矩阵逻辑
Fc=[O I O O O; O O St Rb2t O; O O O O -Rb2t; O O O B1 O; O O O O B2];
Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I];
F=eye(size(Fc))+simdata.Ts*Fc;
G=simdata.Ts*Gc;
end

function [x_out, q_out]=comp_internal_states(x_in,dx,q_in)
R=q2dcm(q_in);
x_out=x_in+dx;
epsilon=dx(7:9);
OMEGA=[0 -epsilon(3) epsilon(2); epsilon(3) 0 -epsilon(1); -epsilon(2) epsilon(1) 0];
R=(eye(3)-OMEGA)*R;
x_out(7)=atan2(R(3,2),R(3,3));
x_out(8)=-atan(R(3,1)/sqrt(1-R(3,1)^2));
x_out(9)=atan2(R(2,1),R(1,1));
q_out=dcm2q(R);
end

function u_out=comp_imu_errors(u_in,x_h)
global simdata;
% 假设只有 bias
u_out=u_in+x_h(10:end);
end

function R=q2dcm(q)
p=zeros(6,1); p(1:4)=q.^2; p(5)=p(2)+p(3);
if p(1)+p(4)+p(5)~=0, p(6)=2/(p(1)+p(4)+p(5)); else, p(6)=0; end
R(1,1)=1-p(6)*p(5); R(2,2)=1-p(6)*(p(1)+p(3)); R(3,3)=1-p(6)*(p(1)+p(2));
p(1)=p(6)*q(1); p(2)=p(6)*q(2); p(5)=p(6)*q(3)*q(4); p(6)=p(1)*q(2);
R(1,2)=p(6)-p(5); R(2,1)=p(6)+p(5);
p(5)=p(2)*q(4); p(6)=p(1)*q(3);
R(1,3)=p(6)+p(5); R(3,1)=p(6)-p(5);
p(5)=p(1)*q(4); p(6)=p(2)*q(3);
R(2,3)=p(6)-p(5); R(3,2)=p(6)+p(5);
end

function q=dcm2q(R)
T = 1 + R(1,1) + R(2,2) + R(3,3);
if T > 10^-8
    S = 0.5 / sqrt(T); qw = 0.25 / S; qx = ( R(3,2) - R(2,3) ) * S; qy = ( R(1,3) - R(3,1) ) * S; qz = ( R(2,1) - R(1,2) ) * S;
else
    if (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt( 1 + R(1,1) - R(2,2) - R(3,3)) * 2; qw = (R(3,2) - R(2,3)) / S; qx = 0.25 * S; qy = (R(1,2) + R(2,1)) / S; qz = (R(1,3) + R(3,1)) / S;
    elseif (R(2,2) > R(3,3))
        S = sqrt( 1 + R(2,2) - R(1,1) - R(3,3) ) * 2; qw = (R(1,3) - R(3,1)) / S; qx = (R(1,2) + R(2,1)) / S; qy = 0.25 * S; qz = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt( 1 + R(3,3) - R(1,1) - R(2,2) ) * 2; qw = (R(2,1) - R(1,2)) / S; qx = (R(1,3) + R(3,1)) / S; qy = (R(2,3) + R(3,2)) / S; qz = 0.25 * S;
    end
end
q = [qx qy qz qw]';
end

function R=Rt2b(ang)
cr=cos(ang(1)); sr=sin(ang(1)); cp=cos(ang(2)); sp=sin(ang(2)); cy=cos(ang(3)); sy=sin(ang(3));
R=[cy*cp sy*cp -sp; -sy*cr+cy*sp*sr cy*cr+sy*sp*sr cp*sr; sy*sr+cy*sp*cr -cy*sr+sy*sp*cr cp*cr];
end

% =========================================================================
% 【新增】 MHT 核心逻辑函数
% =========================================================================
function [decision, mht_state] = run_mht_logic(curr_omega, curr_acc_y, curr_vel_x, mht_state, config)
    % decision 输出: 
    % 0 = 'WAIT' (正在积累数据，不做 HDR)
    % 1 = 'TURN' (确认是转弯，不做 HDR)
    % 2 = 'DRIFT' (确认是漂移，执行 HDR 或 Bias 修正)
    
    decision = 0; % 默认等待
    
    % 1. 获取当前是否在模糊区
    abs_w = abs(curr_omega);
    is_fuzzy = (abs_w > config.gamma_low) && (abs_w < config.gamma_high);
    
    %% 状态 1: 空闲状态 (Idle)
    if ~mht_state.active
        if is_fuzzy
            % === 触发分裂 ===
            mht_state.active = true;
            mht_state.timer = 0;
            mht_state.stable_bias = 0; % 假设当前相对 Bias 为 0 
            
            % 初始化两个分支的累计 Cost
            mht_state.cost_drift = 0;
            mht_state.cost_turn  = 0;
        else
            % 非模糊区：直接根据阈值判断
            if abs_w <= config.gamma_low
                decision = 2; % 极小值，直接视为 Drift/直线 -> 触发 HDR
            else
                decision = 1; % 极大值，直接视为 Turn -> 不触发
            end
        end
        
    %% 状态 2: 分支竞争状态 (Branching)
    else
        mht_state.timer = mht_state.timer + 1;
        
        % --- 计算 Cost (核心逻辑) ---
        
        % 假设 A (Drift): 强制认为 w_true = 0, 所以当前的 w_meas 全部是 Bias 突变
        % Bias 突变惩罚:
        bias_change = abs(curr_omega - mht_state.stable_bias);
        cost_bias = bias_change * config.lambda_bias;
        % Drift 假设下，理论向心力应为 0
        cost_nhc_drift = abs(curr_acc_y - 0) * config.lambda_acc;
        
        mht_state.cost_drift = mht_state.cost_drift + cost_bias + cost_nhc_drift;
        
        % 假设 B (Turn): 认为 w_meas 是真实运动, Bias 没变
        % Bias 突变惩罚 = 0
        % Turn 假设下，理论向心力 = v * w
        expected_ay = curr_vel_x * curr_omega; 
        cost_nhc_turn = abs(curr_acc_y - expected_ay) * config.lambda_acc;
        
        % mht_state.cost_turn = mht_state.cost_turn + cost_nhc_turn;
        base_penalty = 1; % 【新增】给转弯假设加一个基础惩罚
        mht_state.cost_turn = mht_state.cost_turn + cost_nhc_turn + base_penalty;
        
        % --- 判决时刻 ---
        if mht_state.timer >= config.window_size
            if mht_state.cost_turn < mht_state.cost_drift
                decision = 1; % Turn 赢了 -> 不做处理
            else
                decision = 2; % Drift 赢了 -> 需要 HDR
            end
            % 重置状态
            mht_state.active = false;
        end
    end
end