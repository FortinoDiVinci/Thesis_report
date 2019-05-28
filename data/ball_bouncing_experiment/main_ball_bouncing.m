clear all
close all
clc

%% Variables %%

% MACROS -----------------------------------------------------------------
DISP_INIT = 0;

% Initial conditions -----------------------------------------------------
Vb0 = 0;                        % Ball speed (m/s)
Zb0 = 0.55;                     % Ball height (m)
Hp = 0.55;                      % Target height (m)
alpha = 0.48;                   % Restitution coeff paddle/ball
g = 9.81;                       % Gravity acc (SI)
t_max = 15;                     % Trial duration (s)
t_init = 1.72;                  % Init duration of the oscillator (s)
t_s = 0.003;                    % Sampling time (s)
Ar0 = 14;                       % Excitability c
Pt0 = 0.66;                     % Eigen period of the oscillator (s)
p = 0.008;                      % disturbance (m)

% CPG --------------------------------------------------------------------
delay = 16.0;                   % Delay upon the perception of the ball
sigma = 4.4079;                 % Adaptation gain (amplitude)
h0 = 111.5377;                  % Sensor input gain
y_out = 0;                      % Output of the oscillator
x1_out = 0;                     % Init state of the oscillator
x2_out = 1;                     % Init state of the oscillator
f1_out = 0;                     % Init state of the oscillator
f2_out = 1;                     % Init state of the oscillator

% Impedance of the arm ---------------------------------------------------
I = 0.1;                        % Inertia
B = 1.8;                        % Damping
K = 25;                         % Stiffness
h1 = 0.3981;                    % Gain of the torque input
arm_pos_0 = 0.0;                % Arm position along z axis
arm_speed_0 = 0.0;              % Arm speed along z axis

% Storing data variables -------------------------------------------------
if DISP_INIT
    n_step = uint32(t_init / t_s);
else
    n_step = uint32(t_max / t_s);
end
y_out_d = zeros(n_step, 1);
x1_out_d = zeros(n_step, 1);
x2_out_d = zeros(n_step, 1);
f1_out_d = zeros(n_step, 1);
f2_out_d = zeros(n_step, 1);
Zb_d = zeros(n_step,1);                 % ball position along z
Za_d = zeros(n_step,1);                 % arm position along z
Vb_d = zeros(n_step,1);                 % ball speed along z 
Va_d = zeros(n_step,1);                 % arm speed along z

%% Initialization %%

t0 = (0:t_s:t_init);
t_exp = (t_init:t_s:t_max);
t_tot = [t0 , t_exp];

cpg = CPG(Ar0, Pt0, x1_out, x2_out, f1_out, f2_out, t_s, 0);
arm = ARM(K, B, I, arm_pos_0, arm_speed_0, t_s, 0);

for ii=1:length(t0)
    % CPG equation
    matsuoka_output(cpg); 
    x1_out_d(ii) = cpg.x1_out;
    x2_out_d(ii) = cpg.x2_out;
    f1_out_d(ii) = cpg.f1_out;
    f2_out_d(ii) = cpg.f2_out;
    y_out_d(ii) = cpg.y_out;
    arm.Tau = h1*cpg.y_out; %  entrainment of the arm comes from CPG
    Za_d(ii) = arm.pos;
    Va_d(ii) = arm.speed;
    impedance_output(arm);
end

if DISP_INIT
    figure(1)
    grid on ,hold on,
    plot(t0,f1_out_d,'--b',t0,f2_out_d,'--r',t0,x1_out_d,'-b',t0,x2_out_d,'-r','linewidth',1.5) 
    plot(t0,y_out_d,'--k','linewidth',1.5) 
    title('Oscillator initialization')
    legend('f1','f2','x1','x2','y')
    figure(2)
    plot(t0,Za_d,'-b',t0,Va_d,'--r','linewidth',1.5)
    title('Arm initialization entrainment')
    legend('position', 'speed')
end

%% Begining of the experiment %%

% Initialization ---------------------------------------------------------
cpg.t = 0;   
arm.t = 0;
cpg.input = 0;
impact = 0;                                 % used as boolean
t_i = 1;                                    % iteration since last impact
Nb_impact = 0;

for ii=1:length(t_tot)
    % -------------------- Continuous loop ----------------------------- %
    if Nb_impact >= 1                       % starts at 2nd impact
        cpg.input = h0*Vb_d(end-delay,1);   % entrainment of the CPG
    end
    if t_tot(ii) == 10
        position_disturbance(arm, p);
    end
    matsuoka_output(cpg); 
    arm.Tau = h1*cpg.y_out;                 % coupling between arm and CPG
    impedance_output(arm);
    Za_d(ii) = arm.pos; 
    Va_d(ii) = arm.speed;
    % balistic equations
    Zb_d(ii+1) = -0.5*g*t_tot(t_i+1)^2 + Vb0*t_tot(t_i+1) + Zb0; % ball position
    Vb_d(ii) = (Zb_d(ii+1) - Zb_d(ii))/t_s;         % ball speed
    
    % -------------------- Impact detection ---------------------------- %
    if Zb_d(ii+1) <= Za_d(ii)  
        t_i = 1;                            % reset impact time
        impact = 1;                         % boolean
        Nb_impact = Nb_impact + 1;
        Zb0 = Zb_d(ii);                     % ball pos af. impact
        Vb0 = -alpha*(Vb_d(ii) - Va_d(ii)) + Va_d(ii); % ball speed at impact
        Zb_d(ii+1) = -0.5*g*t_tot(t_i+1)^2 + Vb0*t_tot(t_i+1) + Zb0; % theorical ball position
        Zb_d(ii) = max(Zb_d(ii+1), arm.pos);          % real ball position
        Vb_d(ii) = (Zb_d(ii+1) - Zb_d(ii))/t_s;     % ball speed
        
        Pa = 2*Vb0/g;                       % ball period estimation
        ha = (Vb0^2)/(2*g) + Zb0;           % ball apex estimation
    end
    t_i = t_i + 1;
end

if not(DISP_INIT)
    figure(1)
    Zb_d = Zb_d(1:end-1);
    plot(t_tot,Zb_d,'-r',t_tot,Za_d,'--b','linewidth',2.5)
    title('Arm initialization entrainment')
    legend('ball', 'paddle')
end
    
