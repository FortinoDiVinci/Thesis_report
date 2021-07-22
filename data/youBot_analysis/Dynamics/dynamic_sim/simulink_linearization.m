clear all

%% complete control & robot linearisation

init_dynamic_sim
x0 = [th0;dth0;0;0;0;0;0;0;0];
u0 = [0;0;0];

load('linsys_no_secondary_ctrl_2020.mat')

[A,B,C,D] = linmod('dynamic_simulation_control_for_id', x0, u0);
[Ad,Bd,Cd,Dd] = dlinmod('dynamic_simulation_control_for_id', 1e-3, x0, u0);

% [b1,a1] = ss2tf(A,B,C,D,1);
% [b2,a2] = ss2tf(A,B,C,D,2);
% [b3,a3] = ss2tf(A,B,C,D,3);
% 
% H1 = tf(b1,a1);
% H2 = tf(b2,a2);
% H3 = tf(b3,a3);
SStot =  ss(A,B,C,D);
SStotd =  ss(Ad,Bd,Cd,Dd,1e-3);
Htot = tf(SStot);
figure
hold on
bode(SStot(2))
bode(SStotd(2))
%bode(linsys1(2), 'y--')
bode(simulink_linear_continuous_state_sys(2), 'r--')
bode(simulink_linear_discrete_state_sys(2), 'y--')

figure
hold on
% bode(H1)
% bode(H2)
% bode(H3)
bode(Htot(1))
bode(Htot(2))
bode(Htot(3))
%legend('H1','H2','H3','Htot1','Htot2','Htot3')
legend('Htot1','Htot2','Htot3')

% with discrete model (same control as in the real robot)
x0 = [th0;dth0];
u0 = [0;0;0];

[A,B,C,D] = linmod('dynamic_discrete_simulation_control_for_id', x0, u0);
[A1,B1,C1,D1] = linmod('dynamic_discrete_simulation_control_for_id');
[Ad,Bd,Cd,Dd] = dlinmod('dynamic_discrete_simulation_control_for_id', 1e-3, x0, u0);

SS_dyouBot = ss(A,B,C,D);
SS_dyouBot1 = ss(A1,B1,C1,D1);
SS_dyouBotd =  ss(Ad,Bd,Cd,Dd,1e-3);

figure
hold on
bode(SStot(2))
bode(SStotd(2), 'y--')
bode(SS_dyouBotd(2), 'rx')
legend('cont.', 'discr.', 'discr. sim r')

%% youbot dynamics linearisation alone
x0 = [th0;dth0];
u0 = [0;0;0];

[A,B,C,D] = linmod('youBot_dynamics_id_alone', x0, u0);
[Ad,Bd,Cd,Dd] = dlinmod('youBot_dynamics_id_alone', 1e-3, x0, u0);

SS_youBot = ss(A,B,C,D);
SS_youBotd = ss(Ad,Bd,Cd,Dd,1e-3);

figure
hold on
bode(SS_youBot(2))
bode(SS_youBotd(2), 'r--')
%bode(SS_youBotd(2))
'simulink_linear_continuous_state_sys', 'simulink_linear_discrete_state_sys')
%
figure
hold on
bode(SS_youBot(2))
bode(SStot(2))

%% tests

% test with real input

load('..\..\..\ball_bouncing_experiment\experimental_bench_pert\data_2020_Nov_17\data_without_impacts_2020_11_17.mat')
addpath('..\..\..\force_torque_sensor')

exp_nb = 3;

real_x = mocap_marker_robot_base{exp_nb}(:,1);
real_z = mocap_marker_robot_base{exp_nb}(:,3);

[real_f,real_tau,~] = forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
            thetas{exp_nb}', t{exp_nb});

f_in = [-real_f(1,:); -real_f(3,:); -real_tau(2,:)];
        
figure
lsim(Htot,f_in, t{exp_nb})
ylim([-10,10])
