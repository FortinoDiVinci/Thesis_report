clear all
close all
clc

%% Variables %%

% MACROS -----------------------------------------------------------------
DISP_INIT = 0;
DISP_DATA = 0;
SAVE_DATA = 0;
COMP_SAVED_DATA = 1;
MODEL_VERSION = 1;
% 1) Impedance model, cpg entrains the torque of the simulated arm
% 2) K.P. Tee model, cpg entrains the equilibrium position
% 3) Sinusoïd stimuli, with K.P. Tee model
nb_iter = 1;
lstsq_err_data = zeros(nb_iter, 1);
imp_data = zeros(nb_iter, 4);
imp_data_2 = zeros(nb_iter, 4);
for jj = 1:1:nb_iter

disp('Iteration ' + string(jj) + '/' + string(nb_iter));    
    
% Initial conditions -----------------------------------------------------
Vb0 = 0;                        % Ball speed (m/s)
Zb0 = 0.55;                     % Ball height (m)
Hp = 0.55;                      % Target height (m)
alpha = 0.48;                   % Restitution coeff paddle/ball
g = 9.81;                       % Gravity acc (SI)
t_max = 35;                     % Trial duration (s)
t_init = 1.72;                  % Init duration of the oscillator (s)
t_new_h = 20;                   % Time of new target height
t_s_in = 0.03;                  % CPG input entrainment sampling period (s)
% for now t_s_in should be a multiple of t_s
t_s = 0.003;                    % Sampling time (s)
t_1st_ms = 5.5;
t_pert = t_1st_ms + (jj-1)*t_s; % time of a disturbance
Ar0 = 14;                       % Excitability c
Pt0 = 0.66;                     % Eigen period of the oscillator (s)
%p = 0.008;                      % disturbance (m) 
pert_dur = 0.1;                 % duration of the disturbance (s)
tau_e_0 = -2;                   % force disturbance (N)
b_weight = 0.1;                 % ball weight (kg)

% CPG --------------------------------------------------------------------
delay = 36.0/3; %48             % Delay upon the perception of the ball 
lambda = -4.4079;               % Adaptation gain of cpg input
lambda = -3.4;
%lambda = -5;
h0 = 111.5377;                  % Sensor input gain
h0 = 96.54;
% h0 = 6.0;
y_out = 0;                      % Output of the oscillator
x1_out = 0;                     % Init state of the oscillator
x2_out = 1;                     % Init state of the oscillator
f1_out = 0;                     % Init state of the oscillator
f2_out = 1;                     % Init state of the oscillator

% Impedance of the arm ---------------------------------------------------
I = 0.113;                      % Inertia
B = 1.8;                        % Damping
K = 25;                         % Stiffness
h1 = 0.3981;                    % Gain of the torque input
h1 = 0.610;
% h1 = 0.25/Ar0;
h2 = 0.0170;                    % Gain of the equilibrium position input
%h2 = 0.015683644963788;
h2 = 0.0254;
arm_pos_0 = 0.0;                % Arm position along z axis
arm_speed_0 = 0.0;              % Arm speed along z axis
arm_acc_0 = 0.0;                % Arm acceleration along z axis

% Storing data variables -------------------------------------------------
if DISP_INIT
    n_step = uint32(t_init / t_s);
else
    n_step = uint32((t_max - t_init) / t_s);
end
y_out_d = zeros(n_step, 1);
dy_out_d = zeros(n_step, 1);
x1_out_d = zeros(n_step, 1);
x2_out_d = zeros(n_step, 1);
f1_out_d = zeros(n_step, 1);
f2_out_d = zeros(n_step, 1);
Zb_d = zeros(n_step,1);                 % ball position along z
Za_d = zeros(n_step,1);                 % arm position along z
Vb_d = zeros(n_step,1);                 % ball speed along z 
Va_d = zeros(n_step,1);                 % arm speed along z
Tau_d = zeros(n_step,1);                % tau
Tau_ff = zeros(n_step,1);               % feedforward torque for model 2
Tau_e_d = zeros(n_step,1);              % tau induced by the disturbance
acc_d = zeros(n_step,1);

%% Initialization %%

t0 = (0:t_s:t_init);
t_exp = (t_init:t_s:t_max);
t_tot = [t0 , t_exp];
t_1 = (t_init:t_s:t_new_h);
t_2 = (t_new_h+t_s:t_s:t_max);

cpg = CPG(Ar0, Pt0, x1_out, x2_out, f1_out, f2_out, t_s, 0);
arm = ARM(K, B, I, arm_pos_0, arm_speed_0, arm_acc_0, t_s, 0);

disp('Initializing Central Pattern Generator');

for ii=1:length(t0)
    % CPG equation
    matsuoka_output(cpg); 
    x1_out_d(ii) = cpg.x1;
    x2_out_d(ii) = cpg.x2;
    f1_out_d(ii) = cpg.f1;
    f2_out_d(ii) = cpg.f2;
    y_out_d(ii) = cpg.y;
    dy_out_d(ii) = cpg.dy;
    if MODEL_VERSION == 1
        arm.Tau = h1*cpg.y; % entrainment of the arm comes from CPG
        impedance_output(arm);
    elseif MODEL_VERSION == 2
        tee_arm_model(arm, h2*cpg.y, h2*cpg.dy);
        Tau_ff(ii) = arm.tau_ff;
    end
    Za_d(ii) = arm.pos;
    Va_d(ii) = arm.speed;
    Tau_d(ii) = arm.Tau;
    acc_d(ii) = arm.c_acc;
end

if DISP_INIT
    figure(1)
    grid on ,hold on,
    plot(t0,f1_out_d,'--b',t0,f2_out_d,'--r',t0,x1_out_d,'-b',t0,x2_out_d,'-r','linewidth',1.5) 
    plot(t0,y_out_d,'--k','linewidth',1.5) 
    title('Oscillator initialization')
    legend('f1','f2','x1','x2','y')
    figure(2)
    plot(t0,Za_d,'-b',t0,Va_d,'-r','linewidth',1.5)
    title('Arm initialization entrainment')
    legend('position', 'speed')
    
    return;
end

%% Begining of the experiment %%

% Initialization ---------------------------------------------------------
cpg.t = 0;   
arm.t = 0;
cpg.input = 0;
t_i = 1;                                    % iteration since last impact
Nb_impact = 0;
Pa = 1/9.38;
%Zb_d(1) = Zb0; 

%Pt_d = [Pt0];
%Ar_d = [Ar0];
input_cpg_d = [0];

disp('Beginning of the simulated experiment');

for ii=2:length(t_exp)
    % -------------------- Continuous loop ----------------------------- %
    if Nb_impact >= 1                      % starts at 2nd impact
%          if mod(ii, t_s_in/t_s) == 0              % change input every xx ms
%              cpg.input = h0*Vb_d(ii-delay);     % entrainment of the CPG
%          end
        if ii <= delay
            cpg.input = 0;
        else
            cpg.input = h0*Vb_d(ii-delay); % entrainment of the CPG
        end
        input_cpg_d = [input_cpg_d, cpg.input];
    end
    matsuoka_output(cpg); 
    x1_out_d(ii) = cpg.x1;
    x2_out_d(ii) = cpg.x2;
    f1_out_d(ii) = cpg.f1;
    f2_out_d(ii) = cpg.f2;
    y_out_d(ii) = cpg.y;
    dy_out_d(ii) = cpg.dy;

    if ii == length(t_1)
        disp('Changing target height');
        Hp = 0.75;
    end
    
    % --------------------- Disturbance -------------------------------- %
    if t_exp(ii) >= t_pert && t_exp(ii) < t_pert + pert_dur  % 100ms, 4N disturbance
        tau_e = tau_e_0;  % 4
    else
        tau_e = 0;
    end    
    if t_i <= 5 && Nb_impact > 0            % impact last between 5-30ms
        %tau_e = tau_e - b_weight*9.81;% add ball weight 
        %tau_e = 0;
    end 
    if MODEL_VERSION == 1
        arm.Tau = h1*cpg.y + tau_e;     % coupling between arm and CPG
        impedance_output(arm);
    elseif MODEL_VERSION == 2
        arm.tau_e = tau_e;                    % tau = dist in this version          
        tee_arm_model(arm, h2*cpg.y, h2*cpg.dy); 
    elseif MODEL_VERSION == 3
        tee_arm_model(arm, h2*sin(2*pi/Pa*t_exp(ii)), h2*2*pi/Pa*cos(2*pi/Pa*t_exp(ii)));
    end    
    
    Tau_e_d(ii) = tau_e;
    Tau_d(ii) = arm.Tau;
    Za_d(ii) = arm.pos; 
    Va_d(ii) = arm.speed;
    % balistic equations
    Zb_d(ii+1) = -0.5*g*t_tot(t_i+1)^2 + Vb0*t_tot(t_i+1) + Zb0; % ball position
    Vb_d(ii) = (Zb_d(ii+1) - Zb_d(ii))/t_s;         % ball speed
    
    % -------------------- Impact detection ---------------------------- %
    if Zb_d(ii+1) <= Za_d(ii)  
        t_i = 1;                            % reset impact time
        Nb_impact = Nb_impact + 1;
        Zb0 = Zb_d(ii);                     % ball pos af. impact
        Vb0 = -alpha*(Vb_d(ii) - Va_d(ii)) + Va_d(ii); % ball speed at impact
        Zb_d(ii+1) = -0.5*g*t_tot(t_i+1)^2 + Vb0*t_tot(t_i+1) + Zb0; % theorical ball position
        Zb_d(ii) = max(Zb_d(ii+1), arm.pos);          % real ball position
        Vb_d(ii) = (Zb_d(ii+1) - Zb_d(ii))/t_s;     % ball speed
        
        Pa = 2*Vb0/g;                       % ball period estimation
        Ha = (Vb0^2)/(2*g) + Zb0;           % ball apex estimation
    else
        %arm.Tau = 0;
    end
    
    % -------------------- CPG param correction ------------------------ %
    if Nb_impact > 0 && Vb_d(ii) <= 0 && Vb_d(ii - 1) > 0
        err = Ha - Hp;
        if err ~= 0
            cpg.Ar = max(0, cpg.Ar + lambda*err);
%             if sign(err) < 0
%                 disp('Augmentation de Ar')
%             else
%                 disp('Diminution de Ar')
%             end
            %cpg.Pt = max(0.20, Pa); % above 5 Hz the acceleration is too high
            %Ar_d = [Ar_d; cpg.Ar];
        end 
        cpg.Pt = Pa;
        %Pt_d = [Pt_d; cpg.Pt];
    end
    
    t_i = t_i + 1;
end

if not(DISP_INIT) && DISP_DATA
    figure(2)
    hold on, grid on
    Zb_d = Zb_d(1:end-1);
    plot(t_exp,Zb_d,'-r',t_exp,Za_d,'-b','linewidth',2.5)
    %plot(t_exp, Va_d, '-g','linewidth',0.7)   
    target_height = [0.55*ones(1, length(t_1)), 0.75*ones(1, length(t_2))];
    plot(t_exp, target_height, '--k', 'linewidth',1.5)
    t_input = t_exp(length(t_exp) - length(input_cpg_d) + 1:length(t_exp));
    %plot(t_input, input_cpg_d./(4*h0), '-c', 'linewidth',1.0)
    title('Ball bouncing task, (alpha = 0.48, g = 9.81)')
    legend('ball', 'paddle', 'target height')%, 'excitability input/(4*h0)')
    xlabel('(s)'), ylabel('(m)')
    
    figure(3)
    grid on ,hold on,
    plot(t_exp,f1_out_d,'-b',t_exp,f2_out_d,'-r',t_exp,x1_out_d,'--b',t_exp,x2_out_d,'--r','linewidth',1.5) 
    plot(t_exp,y_out_d,'--k','linewidth',1.5) 
    title('Oscillator')
    legend('f1','f2','x1','x2','y')
end
    
if SAVE_DATA == 1
    Zb_saved = Zb_d;
    Za_saved = Za_d;
    Vb_saved = Vb_d;
    Va_saved = Va_d;
    Tau_saved = Tau_d;
    Hp_saved = target_height;
    save('./data/ball_position.mat', 'Zb_saved');
    save('./data/paddle_position.mat', 'Za_saved');
    save('./data/ball_speed.mat', 'Vb_saved');
    save('./data/paddle_speed.mat', 'Va_saved');
    save('./data/torque.mat', 'Tau_saved');
    save('./data/target_height.mat', 'Hp_saved');
    save('./data/params', 'delay', 'lambda', 'h0', 'h1');
end

if COMP_SAVED_DATA == 1    
    load('./data/ball_position.mat');
    load('./data/paddle_position.mat');
    load('./data/ball_speed.mat');
    load('./data/paddle_speed.mat');
    load('./data/torque.mat');
    
    if DISP_DATA
    
        figure(4)
        hold on, grid on
        plot(t_exp,Zb_d,'-r',t_exp,Za_d,'-b','linewidth',1.5);
        plot(t_exp, target_height, '--k', 'linewidth',1.0);
        plot(t_exp,Zb_saved,'--m',t_exp,Za_saved,'--c','linewidth',0.75);
        title('Ball bouncing task, (alpha = 0.48, g = 9.81, p(t=' + string(t_pert) + 's) = '+ string(tau_e_0) + 'N)') %, ball mass = ' + string(b_weight) + ')')
        legend('pert. ball', 'pert. paddle', 'target height', 'ball', 'paddle')
        xlabel('(s)'), ylabel('(m)')

        figure(5)
        hold on, grid on
        yyaxis right
        plot(t_exp, Tau_e_d, '-r', 'linewidth', 1.5);
        ylabel('(Nm)')
        ylim([-4.5, 4.5])
        yyaxis left
        plot(t_exp, Za_d - Za_saved, '-b', 'linewidth', 1.5);
        title('Impedance experimental data')
        legend('displacement', 'force')
        xlabel('(s)'), ylabel('(m)')
    end
    % computation of thetas tilde values, perturbed - reference trajectory
    Za_tild = Za_d - Za_saved;
    Va_tild = Va_d - Va_saved;
    Aa_d = zeros(length(Va_d) - 1, 1);
    Aa_saved = zeros(length(Va_saved) - 1, 1);
    nb_fil = 1;         % nb of pts (bf&af) used to compute derivation
    % acceleration computed using central difference to avoid phase shift
    for ii=nb_fil+1:length(Va_d)-nb_fil
        Aa_d(ii) = (-sum(Va_d(ii-nb_fil:ii-1)) + sum(Va_d(ii+1:ii+nb_fil)))/(2*t_s);
        Aa_saved(ii) = (-sum(Va_saved(ii-nb_fil:ii-1)) + sum(Va_saved(ii+1:ii+nb_fil)))/(2*t_s);
    end
    Aa_tild = Aa_d - Aa_saved;
    Va_tild = Va_tild(1:end-1);
    Za_tild = Za_tild(1:end-1);
    % impedance is computed using a window of 250ms
    idx_1 = intersect(find(t_exp >= t_pert), find(t_exp < t_pert+t_s));
    idx_2 = idx_1 + floor(0.25/t_s);
    phi = [Za_tild(idx_1:idx_2), Va_tild(idx_1:idx_2), Aa_tild(idx_1:idx_2), ones(length(Aa_tild(idx_1:idx_2)),1)];
    %phi2 = [Za_tild(idx_1:idx_2), Va_tild(idx_1:idx_2), ones(length(Aa_tild(idx_1:idx_2)),1)];
    y = Tau_e_d(idx_1:idx_2);    
    % least square algorithm
    impedance = (phi'*phi)\phi'*y;
    %impedance_2 = (phi2'*phi2)\phi2'*y;
    imp_data(jj,:) = impedance';
    %imp_data_2(jj) = impedance_2;
    
    tau_e_res = phi*impedance;
    lstsq_err = sum(abs(y - tau_e_res)); 
    lstsq_err_data(jj) = lstsq_err;
    %tau_e_res_2 = phi2*impedance_2;
    tau_e_th = phi(:,1:3)*[K; B; I];
    
%     figure(6)
%     plot(t_exp(idx_1:idx_2), tau_e_res)
%     hold on, grid on
%     plot(t_exp(idx_1:idx_2), tau_e_th);
%     plot(t_exp(idx_1:idx_2), Tau_e_d(idx_1:idx_2));
%     legend('lstsq', 'arm dyn', 'real')
%     title('Disturbance force reconstruction')
%     xlabel('(s)'), ylabel('(N)')
%     
%     figure(7)
%     plot(t_exp(idx_1:idx_2), Aa_d(idx_1:idx_2), '-r', t_exp(idx_1:idx_2), Aa_saved(idx_1:idx_2), '--m', 'linewidth', 1.0);
%     ylabel('m.s^{-2}')
%     yyaxis right
%     plot(t_exp(idx_1:idx_2), Va_d(idx_1:idx_2), '-b', t_exp(idx_1:idx_2), Va_saved(idx_1:idx_2), '--c', 'linewidth', 1.0);
%     ylabel('m.s^{-1}')
%     legend('acc', 'ref. acc.', 'speed', 'ref. speed');
%     title('Speed and computed acceleration of both reference and perturbed simulation');
end
% figure(4)
% grid on ,hold on,
% plot(t_exp,y_out_d,'-b','linewidth',1.5) 
% plot(t_exp,dy_out_d,'--k','linewidth',1) 
% title('Oscillator')
% legend('y','dy')

end

%% Evolution of impedance
% plot the impedance during all the measuments done, starting at the first
% perturbation and ending at the last one

t_dif_pert = (1:1:nb_iter);
t_dif_pert = (t_dif_pert-1)*t_s + t_1st_ms;

idx_1b = intersect(find(t_exp >= t_dif_pert(1)), find(t_exp < t_dif_pert(1) + t_s));
idx_2b = intersect(find(t_exp >= t_dif_pert(end)), find(t_exp < t_dif_pert(end) + t_s));

figure(6)
subplot(3,1,1);
plot(t_dif_pert, imp_data(:,1)', '-x')
ylabel('Stiffness (N/m)')
yyaxis right
plot(t_dif_pert, imp_data(:,2)', '-x')
ylabel('Damping (N.s/m)')
legend('Stiffness', 'Damping')
title('Impedance Stiffness & Damping evolution')
subplot(3,1,2);
plot(t_dif_pert, imp_data(:,3)', '-x')
ylabel('Inertia (N.s^2/m)')
legend('Inertia')
title('Impedance Inertia evolution')
subplot(3,1,3);
plot(t_exp(idx_1b:idx_2b), Za_saved(idx_1b:idx_2b), '-')
legend('Arm')
title('Position')
xlabel('time (s)')
ylabel('height (m)')

figure(7)
plot(y)
hold on, grid on 
plot(tau_e_res)
title('Last estimation fit')

figure(8)
plot(t_exp(idx_1b:idx_2b), lstsq_err_data, 'r-x')
legend('LSTSQ sum remainder')
title('Identification reliability')
xlabel('time (s)')
ylabel('torque (Nm)')