clear all
close all

addpath('../../../force_torque_sensor')
addpath('../../../youBot_analysis/Utils')

load('data_vfo_3_phases.mat')

kinematic_coeff = 6;
virtual_pos_offset = -0.32;

Time = [];
Force = [];
Position = [];
Ball = [];
TrialNb = [];
Hand = [];
IsBallImpact = [];

j = 1;
for it = 1:length(folder_names)

    [f,~,~]= forces_filtering(forces_unf{it}', torques_unf{it}', ...
        thetas{it}', t{it});
    
    Time = [Time; t{it}];
    Force = [Force; -f(3,:)'];
    Position = [Position; mocap_marker_robot_base{it}(:,3)];
    Ball = [Ball; z_b{it}./kinematic_coeff - virtual_pos_offset];
    TrialNb = [TrialNb; j*ones(size(z_b{it}))];
    Hand = [Hand; zeros(size(z_b{it}))]; % only left hand was used    
    IsBallImpact = [IsBallImpact; ones(size(z_b{it}))]; % True
    
    j = j + 1;
    
end

T = table(Time, Position, Force, Ball, Hand, IsBallImpact, TrialNb);
writetable(T,'data_vfo_3_phases_concatenated.csv')