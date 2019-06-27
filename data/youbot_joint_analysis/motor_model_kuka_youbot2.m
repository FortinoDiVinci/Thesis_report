clear all, close all

%% EC 45 FLAT 70W

R = 0.608;                  % Terminal resistance (Ohm)
L = 0.463e-3;               % Terminal inductance (H)
J = 181e-6;                 % Rotor inertia (kg.m^2)
Kt = 36.9e-3;               % Torque constant (Nm/A)
tau_m = 8.07e-3;            % Mechanical time constant (s)
kf = R*J/L * 1e-3;          % friction (considered negligeable)

tau_e = L/(3*R);
Ke = 3*R*J/(tau_m*Kt);

K_m = 31e-1;                % Stiffness between motor and joint
B_m = 23e-3;                % Damping between motor and joint
J_rob = 661e-6;             % Inertia of the robot joint

K_env = 1;                  % Stiffness of a simulated env (N/m)
B_env = 1e-2;               % Damping of a simùulated env (N.s/m)

SYMBOL = 0;                 % For symbolic computation set to 1
MAPPING = 0;                % For brut force mapping of the stability
ROUTH = 0;                  % Use routh criterion if set to 1
N = 10;                     % max value of K and Ki to evaluate

%% Correctors

if SYMBOL == 1
    syms k ki EPS s;
else
    k = 10;
    ki = (1/J_rob)*1.7;
end
K = k;                      % Proportional gain for corrector
Ki = ki;                    % Integral gain for corrector

Kv = 200/256;               % Speed loop gain

K_cur = 3000/256;           % Proportional gain for current corrector
Ki_cur = 3000/262144;       % Integral gain for current corrector

%% Analysis

p = tf('p');

% Current loop
%I_pi = (K_cur*p + Ki_cur)/p;
%I_ = 1/(1e-3*p + 1);
%I_loop = feedback(I_pi * I_, 1);
I_loop = 1/(1e-3*p + 1);
% I_loop = 1/(1+

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

if SYMBOL == 1 && MAPPING ==0
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
    
elseif SYMBOL == 0 && MAPPING ==0
    Impedance_pi = (K*p + Ki)/p;
    TF_mot = TF_mot_nd*Impedance_pi + TF_mot_ni;
    figure(3)
    step(TF_mot);
end

if MAPPING == 1
    map = zeros(3, N^2);
    syms EPS;
    if ROUTH == 0
        figure(3)
        hold on, grid on
        title('Poles & Zeros')
    end
    for i=1:N
        for j=1:N
            K = i/1000;
            Ki = j/1000;
            Impedance_pi = (K*p + Ki)/p;
            TF_mot = TF_mot_nd*Impedance_pi - TF_mot_ni;
            
            num_TF_mot = TF_mot.num{1};
            den_TF_mot = TF_mot.den{1};
            
            if ROUTH == 1
                r = routh(den_TF_mot, EPS);
                n = length(r(:,1));
                nb = N*(i-1)+j;
                count = 0;
                zer = zeros(n,1);

                if (r(:,1) > zer)
                    map(:,nb) = [i;j;1];
                else
                    map(:,nb) = [i;j;0];
                end
            % ELSE PLOT POLES & ZEROS
            else
                plot(roots(den_TF_mot), 'x'); 
                plot(roots(num_TF_mot), 'o');
            end 
        end
    end
end

%r_num = routh([TF_mot.num{1}],EPS);
%r_den = routh([TF_mot.den{1}],EPS);

