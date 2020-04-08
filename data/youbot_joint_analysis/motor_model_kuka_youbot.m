%% EC 45 FLAT 50W for joint 1 to 3

JOINT = 3

if JOINT <=3

    R = 0.978;                          % Terminal resistance (Ohm)
    L = 0.573;                          % Terminal inductance (H)
    Jm = 13.5e-6;                       % Rotor inertia (Kgm^2)
    Kt = 33.5e-3;                       % Torque constant (Nm/A)
    Ke = 33.5e-3;                       % Speed constant (V/(rad/s))
    tau_m = 11.8e-3;                    % Mechanical time constant (s)
    kf = R*Jm/L * 1e-6;                 % friction (considered negligeable)

    tau_e = L/(3*R);
    %Ke = 3*R*Jm/(tau_m*Kt);
    m = (110 + 75 + 46 + 821 + 769 + 687 + 162)*10^-3;
    N = 100;                    % gear reduction ratio
    Jr = 0.071e-6;              % gear inertia (Kg.m^2) 
    Jch = m*((10.3 + 57.16 + 113.6 + 135)*10^-3/2)^2;  % computed inertia when robot is straight
    J = Jm + 1/(N^2)*(Jr + Jch);
 
    weight = m*[0;0;-g];
    %w_pert = dot(m*[0;0;-g], [sin(theta);0;cos(theta)]);

    % actual values
    K = 0.4;
    Ki = 5;
    Kv = 4000/256;
    Kvi = 1000/65536;
    K_cur = 1500/256;
    Ki_cur = 1500/262144;
    K_pos = 200/256;
    
elseif JOINT == 4

    R = 4.48;                       % Terminal resistance (Ohm)
    L = 2.24;                       % Terminal inductance (H)
    Jm = 9.25e-6;                   % Rotor inertia (Kgm^2)
    Kt = 51e-3;                     % Torque constant (Nm/A)
    Ke = 51.1e-3;                   % Speed constant (V/(rad/s))
    tau_m = 11.8e-3;%8.07e-3;       % Mechanical time constant (s)
    kf = R*Jm/L * 1e-6;             % friction (considered negligeable)

    tau_e = L/(3*R);
    %Ke = 3*R*Jm/(tau_m*Kt);
    m = (75 + 46 + 769 + 687 + 162)*10^-3;
    N = 71;                         % gear reduction ratio
    Jr = 0.07e-6;                   % gear inertia (Kg.m^2) 
    Jch = m*((10.3 + 57.16 + 113.6)*10^-3/2)^2;  % computed inertia when robot is straight
    J = Jm + 1/(N^2)*(Jr + Jch);

    weight = m*[0;0;-g];
    %w_pert = dot(m*[0;0;-g], [sin(theta);0;cos(theta)]);

    % actual values
    K = 0.4;
    Ki = 5;
    Kv = 4000/256;
    Kvi = 1000/65536;
    K_cur = 1500/256;
    Ki_cur = 1500/262144;
    K_pos = 200/256;    
   
end
    
% simulated env
K_env = 25;                  % Stiffness of a simulated env (N/m)
B_env = 1.8;                 % Damping of a simùulated env (N.s/m)
I_env = 0.1;

SYMBOL = 0;                 % For symbolic computation set to 1

%% Corrector

if SYMBOL == 1
    syms k ki EPS s;
else
    k = 10;
    ki = 1;
end
K = k;                      % Proportional gain for corrector
Ki = ki;                     % Integral gain for corrector

Kv = 200;                   % Speed loop gain

K_cur = 3000;               % Proportional gain for current corrector
Ki_cur = 3000;              % Integral gain for current corrector

%% Analysis

p = tf('p');

% Current loop
I_pi = (K_cur*p + Ki_cur)/p;
I_ = 1/(1e-3*p + 1);
I_loop = feedback(I_pi * I_, 1);

% Speed loop without disturbance
H_mot = 1/(J*p + kf);
TF_mot_nd = feedback(H_mot*Kt*I_loop*Kv, 1);
figure(1)
step(TF_mot_nd);

% Speed loop without input
TF_mot_ni = feedback(H_mot, Kv*I_loop*Kt);
figure(2)
step(TF_mot_ni);

% Complete loop

if SYMBOL == 1
    num_TF_mot_nd = TF_mot_nd.num{1};
    den_TF_mot_nd = TF_mot_nd.den{1};
    num_TF_mot_ni = TF_mot_ni.num{1};
    den_TF_mot_ni = TF_mot_ni.den{1};

    num_c_TF_mot_nd = [num_TF_mot_nd*K, 0] + [0, num_TF_mot_nd*Ki];
    den_c_TF_mot_nd = [den_TF_mot_nd, 0];
   
    TF_mot_nd_syms = poly2sym(num_c_TF_mot_nd, s)/poly2sym(den_c_TF_mot_nd, s);
    TF_mot_ni_syms = poly2sym(num_TF_mot_ni, s)/poly2sym(den_TF_mot_ni, s);
    
    TF_mot_syms = TF_mot_nd_syms + TF_mot_ni_syms;
    
    [TF_mot_syms_num, TF_mot_syms_den] = numden(TF_mot_syms);
    
    [coef_num, ~] = coeffs(TF_mot_syms_num, s, 'all');
    [coef_den, ~] = coeffs(TF_mot_syms_den, s, 'all');
    
    mot_num_r = routh(coef_num, EPS);
    mot_den_r = routh(coef_den, EPS);
    
    simplify(mot_num_r)
    simplify(mot_den_r)
    
elseif SYMBOL == 0
    Impedance_pi = (K*p + Ki)/p;
    TF_mot = TF_mot_nd*Impedance_pi + TF_mot_ni;
    figure(3)
    %step(TF_mot);
    [u,t] = gensig('square',4,10,0.1);
    lsim(TF_mot,u,t)
end
%r_num = routh([TF_mot.num{1}],EPS);
%r_den = routh([TF_mot.den{1}],EPS);

