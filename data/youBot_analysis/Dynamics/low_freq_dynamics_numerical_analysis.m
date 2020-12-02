% Low dynamic evaluation for q0

q0_kuka = [1.676; -4.363; 1.497];
q_dh = [-90 0 90]'*pi/180;        % Denavit H. 
q_rob = [65 -146 102.5]'.*pi/180; % robot offsets
q0 = (q_rob - q_dh) - q0_kuka;    % simulation convention
dq0 = [0; 0; 0];
tau0 = G_3DOF(q0);
tau0_exp = [2;2;1.3;]; % a quick evaluation was done, this needs to be confimed
% admittance controller gains
Kp_f = 0.015;
Ki_f = 0.08;
% velocity controller gains
Kp_v = zeros(3,3); Kp_v(1,1)=2500; Kp_v(2,2)=1500; Kp_v(3,3)=2000; 
Kp_v = Kp_v./256;
Ki_v = zeros(3,3); Ki_v(1,1)=3000; Ki_v(2,2)=900; Ki_v(3,3)=1000; 
Ki_v = Ki_v./65536;
% dynamic evaluation at q0
iM = inv(M_3DOF(q0(2:3)));
J0E = J0E_3DOF(q0(1:3));
J = J0E([1,3,5],:);
iJ = inv(J);
syms q1 q2 q3;
Ksym = jacobian(inv(M_3DOF([q2;q3]))*tau0, [q1;q2;q3]) + ...
    jacobian(inv(M_3DOF([q2;q3]))*G_3DOF([q1;q2;q3]), [q1;q2;q3]);
K = double(vpa(subs(Ksym,[q1;q2;q3],q0)));

A = J*iM*Ki_v*iJ*Ki_f;
B = J*(iM*Ki_v*iJ + K*iJ);

s = tf('s');
LD_gain = B\A;

LD1z = LD_gain(2,2)/(s); % vz = LDz * fz
LD2z = LD_gain(2,2)/(s^2); % z = LDz * fz

% Impedance system: human arm
M = 0.2;
B = 5;
K = 50;
H_imp = M*s^2 + B*s + K;

% input 
t = 0:0.001:10; 
f = 0.8; %  the analysis is accurate for frequency close to 0
A = 5;
u = A*sin(2*pi*f.*t + 3*pi/2);

figure
lsim(LD1z,u,t)

figure
lsim(LD2z,u,t)

figure
bode(LD2z)

% frequence de coupure à 0.03 Hz

%% 
% Conséquence de la variation de Ki_f, le gain intégrale du controlleur 
% en admittance

Ki_f = (0:0.001:0.1);

A = J*iM*Ki_v*iJ;
B = J*(iM*Ki_v*iJ + K*iJ);
LD_gain = (B\A);
LD_zgain = LD_gain(2,2).*(Ki_f);

figure('DefaultAxesFontSize',14)
plot(Ki_f, 1./LD_zgain)
xlabel('Ki gain')
ylabel('Masse apparente (kg)')