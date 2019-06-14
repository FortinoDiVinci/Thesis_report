%% EC 45 FLAT 70W

R = 0.608;                  % Terminal resistance (Ohm)
L = 0.463e-3;               % Terminal inductance (H)
J = 181e-6;                 % Rotor inertia (Kgm^2)
Kt = 36.9e-3;               % Torque constant (Nm/A)
tau_m = 8.07e-3;            % Mechanical time constant (s)
kf = R*J/L * 1e-3;          % friction (considered negligeable)

tau_e = L/(3*R);
Ke = 3*R*J/(tau_m*Kt);

SYMBOL = 1;                 % For symbolic computation set to 1

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
    step(TF_mot);
end
%r_num = routh([TF_mot.num{1}],EPS);
%r_den = routh([TF_mot.den{1}],EPS);

