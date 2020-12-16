clear all

init_dynamic_sim

[A,B,C,D] = dlinmod('dynamic_simulation_control_for_id');

% [b1,a1] = ss2tf(A,B,C,D,1);
% [b2,a2] = ss2tf(A,B,C,D,2);
% [b3,a3] = ss2tf(A,B,C,D,3);
% 
% H1 = tf(b1,a1);
% H2 = tf(b2,a2);
% H3 = tf(b3,a3);
Htot = tf(ss(A,B,C,D,dt));

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
