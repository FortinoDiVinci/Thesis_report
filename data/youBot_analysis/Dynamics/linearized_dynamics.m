% Linear simulation
clear all

addpath('dynamic_sim/')

% parameters
K = 0.015;
Ki = [0.001;0.01;0.08;0.4;2];

Kv = [2500;1500;2000]/256;
Kvi = [3000;900;1000]/65536;

q0_kuka = [1.676; -4.363; 1.497];
q_dh = [90 0 -90]'*pi/180;
offset = [65 -146 102.5]'.*pi/180;
%q0 = q0_kuka - offset;
th0 = q_dh - (q0_kuka - offset);
dth0 = [0;0;0];

tau0 = G_3DOF(th0(1),th0(2),th0(3));

% For this analysis, the inertia, jacobian and coriolis matrices are
% evaluated at the configuration q0 after a linearization at this neigborhood
iM = iM_3DOF(th0(2),th0(3));
J = J0E_3DOF(th0(1),th0(2),th0(3));
iJ = iJ0E_3DOF(th0(1),th0(2),th0(3));
Fv = zeros(3,3); % TODO: assign value

s = tf('s');

I = eye(3);
Cv = tf(zeros(3)); 
C  = tf(zeros(3));
C(2,2) = 1;
for i = 1:3
    Cv(i,i) = Kv(i) + Kvi(i)/s;
end

syms th1 th2 th3 dth1 dth2 dth3;
Ksym = jacobian(iM_3DOF(th2,th3)*tau0, [th1;th2;th3]) + ...
    jacobian(iM_3DOF(th2,th3)*G_3DOF(th1,th2,th3), [th1;th2;th3]);
Kd = double(vpa(subs(Ksym,[th1;th2;th3],th0)));

Vsym = -(Fv - jacobian(C_3DOF(dth1,dth2,dth3,th2,th3)*[dth1;dth2;dth3], [th1;th2;th3]));
Vd = double(vpa(subs(Vsym,[dth1;dth2;dth3;th1;th2;th3],[dth0;th0])));

Jdotsym = diff(J0E_3DOF(th1,th2,th3),th1,th2,th3);
Jdot = double(vpa(subs(Jdotsym,[th1;th2;th3],th0)));

% see computation
for it = 1:5
    Ci = C*(K + Ki(it)/s);
    A{it} = J*iM*(Cv + iJ*Ci + J');
    B{it} = s*I + (J*iM *(Cv - Vd) - Jdot)*iJ - J*Kd*iJ/s;
    H_rob{it} = B{it}\A{it};
    Hz_rob{it} = H_rob{it}(2,2);
end

figure
bode(Hz_rob{3},Hz_rob{3}/s)
legend('vz/fz','z/fz')

[b,a] = tfdata(Hz_rob{3}/s);
ssHz_rob = tf2ss(b{1},a{1});
zpk(Hz_rob{3}/s)

%%
A_ld = J*iM*diag(Kvi)*iJ;
B_ld = J*(iM*diag(Kvi) - Kd)*iJ;
LD_gain = (B_ld\A_ld); % low dynamics gains
for it = 1:5
    LD_zgain{it} = LD_gain(2,2)*Ki(it);
    LD_Hz_rob{it} = LD_zgain{it}/(s);
end

figure
hold on
opts = bodeoptions;
opts.FreqUnits = 'Hz';
for it = 1:5
    bodeplot(Hz_rob{it}/s, opts)
end
str_col = convertStringsToChars(["b","r","y","m","g"]+"--");
for it = 1:5
    bodeplot(LD_Hz_rob{it}/s, str_col{it}, opts)
end
legend(num2str([Ki;Ki]))