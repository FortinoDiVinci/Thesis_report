%% Dynamic script for the admittance control loop 
clear all
close all

addpath('../utils/')

%% Initialization
% time
dt = 1e-3;
t_end = 15;
t = 0:dt:t_end;
% joints
q0_kuka = [1.676; -4.363; 1.497];
q_dh = [-90 0 90]'*pi/180;       % Denavit H. 
q_rob = [65 -146 102.5]'*pi/180; % robot offsets
q0 = (q_rob - q_dh) - q0_kuka;  % simulation convention
dq0 = [0; 0; 0];
% admittance controller gains
Kp_f = 0.015;
Ki_f = 0.08;
Ti_f = 1/Ki_f;
adm_ctrl = PI(dt, Kp_f, Ki_f);
% velocity controller gains
Kp_v = [2500;1500;2000]./256;
Ki_v = [3000;900;1000]./65536;
Ti_v = 1./Ki_v;
vel_ctrl = PI(dt, Kp_v, Ki_v);
% Environnement var
Mz = 0.1;
Bz = 5;
Kz = 150;
ddz0 = 0;
dz0 = 0;
z0 = 0.4;
% inputs
fz_in = 0*sin(2*pi*0.9.*t); % environnement force
f0 = zeros(size(fz_in)); % admittance force set point
fz_env = 0;
% recorded data
data.dynamic.fz_in = fz_in;
% data.kinematic.joints.q(:,1) = q0;
% data.kinematic.joints.dq(:,1) = dq0;
% data.kinematic.joints.ddq(:,1) = [0;0;0];
% data.kinematic.cartesian.z(1) = NaN; % TODO init
% data.kinematic.cartesian.dz(1) = NaN; % TODO init
% data.kinematic.cartesian.ddz(1) = NaN; % TODO init
%%
q = q0;
dq = dq0;
for ii = 1:length(t)
    J0E = J0E_3DOF(q);
    f_tot = [0;0;fz_in(ii)-fz_env;0;0;0];
    % Admittance control
    adm_cmd = adm_ctrl.compute(f0(ii) - f_tot(3));
    % Cartesian space to joint space
    vel_setp = J0E([1,3,5],:)\[0;adm_cmd;0]; % cmd is only along z axis
    % Joint velocity control
    vel_cmd = vel_ctrl.compute(vel_setp - dq);
    % Joint current control is neglected
    tau = vel_cmd; % vel_cmd can also be seen as the torque set points
    % Robot Dynamic model
    [ddq, dq, q] = DynModel_3DOF(f_tot,tau,q,dq,dt);
    % Joint space to cartesian space
    hm = DGM_3DOF(q); % homogenous tranformation matrix
    cartesian_velocity = J0E_3DOF(q)*dq;
    dJ0E = ((J0E_3DOF(q)-J0E)./dt); % derivative of the jacobian
    cartesian_acceleration = J0E_3DOF(q)*ddq + dJ0E*dq;    
    
    % Data recording
    data.kinematic.cartesian.z(ii) = hm(3,4);
    data.kinematic.cartesian.dz(ii) = cartesian_velocity(3);
    data.kinematic.cartesian.ddz(ii) = cartesian_acceleration(3);
    data.kinematic.joints.q(:,ii) = q;
    data.kinematic.joints.dq(:,ii) = dq;
    data.kinematic.joints.ddq(:,ii) = ddq;
    % Reaction force of the environment
    fz_env = 0;%Mz*(data.kinematic.cartesian.ddz(ii) - ddz0) + ...
     %Bz*(data.kinematic.cartesian.dz(ii) - dz0) + ...
     %Kz*(data.kinematic.cartesian.z(ii) - z0);  
    data.dynamic.f_env(:,ii) = [0;0;fz_env];   
end

%%

%simulateYouBot_3DOF(data.kinematic.joints.q,data.dynamic.f_env)

% 
figure
plot(t, data.kinematic.cartesian.z)

figure
plot(t, data.dynamic.f_env(3,:))