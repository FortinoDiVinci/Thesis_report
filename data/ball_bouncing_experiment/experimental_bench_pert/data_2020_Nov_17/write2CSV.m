clear all
close all

addpath('../../../force_torque_sensor')
addpath('../../../youBot_analysis/Utils')

load('data_without_impacts_2020_11_17.mat')

Time = [];
Force = [];
Position = [];
Ball = [];
TrialNb = [];
Hand = [];
IsBallImpact = [];

j = 1;
for it = 1:length(folder_names)
    
    if contains(folder_names(it), "calibration")
        continue
    end
    
    [f,~,~]= forces_filtering(forces_unf{it}', torques_unf{it}', ...
        thetas{it}', t{it});
    
    Time = [Time; t{it}];
    Force = [Force; -f(3,:)'];
    Position = [Position; mocap_marker_robot_base{it}(:,3)];
    Ball = [Ball; z_b{it}./kinematic_coeff(fld_idx) - virtual_pos_offset];
    TrialNb = [TrialNb; j*ones(size(z_b{it}))];
    if (rem(j,2) == 1)
        Hand = [Hand; zeros(size(z_b{it}))]; % start with left hand
    else
        Hand = [Hand; ones(size(z_b{it}))];
    end
    IsBallImpact = [IsBallImpact; zeros(size(z_b{it}))]; % False
    
    j = j + 1;
    
end

load('data_with_impacts_2020_11_17.mat')

j = 15;
for it = 1:length(folder_names)
    
    if contains(folder_names(it), "calibration")
        continue
    end
    
    [f,~,~]= forces_filtering(forces_unf{it}', torques_unf{it}', ...
        thetas{it}', t{it});
    
    Time = [Time; t{it}];
    Force = [Force; -f(3,:)'];
    Position = [Position; mocap_marker_robot_base{it}(:,3)];
    Ball = [Ball; z_b{it}./kinematic_coeff(fld_idx) - virtual_pos_offset];
    TrialNb = [TrialNb; j*ones(size(z_b{it}))];
    if (rem(j,2) == 1)
        Hand = [Hand; ones(size(z_b{it}))]; % start with right hand
    else
        Hand = [Hand; zeros(size(z_b{it}))];
    end
    IsBallImpact = [IsBallImpact; ones(size(z_b{it}))]; % True
    
    j = j + 1;
    
end

T = table(Time, Position, Force, Ball, Hand, IsBallImpact, TrialNb);
writetable(T,'data_2020_Nov_17_concatenated.csv')