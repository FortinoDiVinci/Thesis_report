%addpath(genpath('C:/Users/Fortineau_Vin/Documents/MATLAB/yalmip'))

%load('dyn_model_ss_b.mat')
%load('../Dynamics/robot_model_endpoint_input_output.mat','dyn_model_ss_b');

% Robot model with two input (u (cmd) and fz (measure)), and a single
% output the velocity about the z axis
% Sigma_f = dyn_model_ss_b(1);
% Sigma_v = dyn_model_ss_b(3);
% 
% A_sig = [Sigma_v.A, zeros(size(Sigma_v.A,1), size(Sigma_f.A,2));
%          zeros(size(Sigma_f.A,1), size(Sigma_v.A,2)), Sigma_f.A];
% B_sig = [Sigma_v.B, zeros(size(Sigma_v.B,1), size(Sigma_f.B,2));
%          zeros(size(Sigma_f.B,1), size(Sigma_v.B,2)), Sigma_f.B];
% C_sig = zeros(1, length(A_sig));
% C_sig(5) = Sigma_v.C(5); % only the velocity on z is considered (controlled output)
% 
% Sigma = ss(A_sig, B_sig, C_sig, zeros(1,2));

load('../Dynamics/youbot_lin_model_z.mat');

Sigma = youbot_lin_model_ss_z;

syms k b m tau
k0 = 200; b0 = 10; m0 = 0.5; tau0 = 1/(10*2*pi);

A_env = [0     1        0   ;
         0     0        1   ;
         0 -1/(tau^2) -2/tau];
B_env = [0; 0; 1];
C_env = [-k, -b, -m]./(tau^2);
D_env = 0;

% A_env0 = double(subs(A_env, tau, tau0));
% B_env0 = B_env;
% C_env0 = double(subs(C_env, [k,b,m,tau], [k0,b0,m0,tau0]));
% D_env0 = D_env;
% % 
% H_env = ss(A_env0, B_env0, C_env0, D_env0);
s = tf('s');
H_filt = 1/(1 + 1/(2*pi*1000)*s); % 1kHz 1st order filter 
H_filt_ss = ss(H_filt);

% low pass filtered environment, to avoid having impedance parameters in
% the C matrix
A_env_f = [H_filt_ss.A, H_filt_ss.B*C_env;
            zeros(3,1),         A_env    ];
B_env_f = [0;B_env];
C_env_f = [H_filt_ss.C, zeros(1,3)];
D_env_f = 0;

% H_env_tf = tf(H_env);

% H_env_cmp = (k0 + b0*s + m0*s^2)/(s*(1 + tau0*s)^2);
% figure
% bode(H_env, H_env_cmp, '--')

% coupling between youBot dynamic model and environment

A = [A_env_f, B_env_f*Sigma.C;
    zeros(9,4), Sigma.A ];
B = [zeros(4,2); Sigma.B];
C = [C_env_f, zeros(1, 9)];
Bu = B(:,1); % cmd
Bw = B(:,2); % perturbation

Cz = [0, 0, 1, zeros(1, 10)]; % velocity on z
% 
% A0 = double(subs(A, [tau,k,b,m], [tau0,k0,b0,m0]));
% B0 = B;
% C0 = Cz;
% D0 = zeros(1,2);
% H0 = ss(A0,B0,C0,D0);
% 
% figure
% bode(H0(1,1))
% 
% figure
% step(H0)

%% snippet

n = length(A);
m_u = size(Bu,2);

Q = sdpvar(n,n); % Lyapunov function
Y = sdpvar(m_u,n); % K = Y*pinv(Cy*Q)
gamma = sdpvar(1,1); 

eps = 1e-6;

inequalities = [];
inequalities = [Q >= eps];
inequalities = [inequalities; gamma >= eps];

min_k = 50;
max_k = 450;
min_b = 5;
max_b = 15;
min_m = 0.3;
max_m = 0.7;

m_val = [min_m, min_m, min_m, min_m, max_m, max_m, max_m, max_m];
k_val = [min_k, min_k, max_k, max_k, min_k, min_k, max_k, max_k];
b_val = [min_b, max_b, min_b, max_b, min_b, max_b, min_b, max_b];

for i = 1 : 8
    A_i = double(subs(A, [k, b, m, tau], [k_val(i), b_val(i), m_val(i), tau0]));
    
    tmp_mat = [A_i*Q + Q*A_i' + Bu*Y + Y'*Bu',  Bw    ,             Q*Cz'             ;
                     Bw'       , -gamma*eye(size(Bw,2)),  zeros(size(Bw,2), size(Cz, 1));
                     Cz*Q      , zeros(size(Cz, 1), size(Bw,2)), -gamma*eye(size(Cz,1))];
     
    inequalities = [inequalities; tmp_mat <= -eps];
end
% 
cost = gamma;
% 
options = sdpsettings('solver','SDPT3-4','verbose',1,'debug',1);
% install_sdpt3
optimize(inequalities, cost, options);
% 
Y_val = value(Y);
Q_val = value(Q);
gamma_val = value(gamma);

Cor_meas = Y_val*pinv(C*Q_val); % u = K * Cy
Cor_sr = Y_val/Q_val;           % u = K * X
