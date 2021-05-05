% dynamic youBot model validation
clear all
addpath('../../../utils')
addpath('../../../force_torque_sensor')
addpath('../dynamic_sim/')

load('..\..\..\ball_bouncing_experiment\experimental_bench_pert\data_2020_Nov_17\data_without_impacts_2020_11_17.mat', ...
    't', 'joint_eff', 'thetas', 'dt', 'forces_unf', 'torques_unf')
exp_nb = 2;

% real data
time = t{exp_nb};
tau_m = joint_eff{exp_nb}(:,2:4);
q_m = thetas{exp_nb}(:,2:4);
[f_tmp, tau_tmp] = forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t{exp_nb});

% joints
q0_kuka = [1.676; -4.363; 1.497];
q_dh = [90 0 -90]'*pi/180;       % Denavit H. 
q_rob = [65 -146 102.5]'*pi/180; % robot offsets
th0 = q_dh - (q0_kuka - q_rob);  % simulation convention

% data filter
[b,a] = butter(2,50/(1/(2*dt)),'low'); % 2nd order 50Hz low pass filter
f_filt = filtfilt(b,a, f_tmp')';
tau_filt = filtfilt(b,a, tau_tmp')';
fe = [f_filt(1,:); f_filt(3,:); tau_filt(2,:)]; % [fx;fz;fth]

%f_tot = [0,0,0]';
th = th0;
dth = [0;0;0];

% direct dynamic main loop
for ii = 1:length(time)

    %tau = cmd_vel(ii,:).*Tc.*R;
    tau = tau_m(ii,:)';
    [ddth, dth, th] = DynModel_3DOF(fe(:,ii),tau,th,dth,dt);
    
    th_s(:,ii) = th;
    dth_s(:,ii) = dth;
    ddth_s(:,ii) = ddth;

end

th_m = q_dh - (q_m' - q_rob);
th_m = filtfilt(b,a,th_m')';

for ii = 1:size(th_m,1)
    dth_m(ii,:) = Iu_diffcent(th_m(ii,:)', time)';
    %dth_m(ii,:) = filtfilt(b,a,dth_m(ii,:)')';
    ddth_m(ii,:) = Iu_diffcent(dth_m(ii,:)', time)';
    %ddth_m(ii,:) = filtfilt(b,a,ddth_m(ii,:)')';
end

f = waitbar(0,'1','Name','Simulating youBot dynamics',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');

setappdata(f,'canceling',0);

% inverse dynamic main loop
for ii = length(time):-1:1

    if mod(ii,1e3) == 1 % otherwise slows down the computation
        if getappdata(f,'canceling')
            break
        end
        waitbar((length(time)-ii)/length(time),f,sprintf('%.2f%%',(length(time)-ii)/length(time)*100))
    end
    th = th_m(:,ii);
    dth = dth_m(:,ii);
    ddth = ddth_m(:,ii);
    
    %tau_s_noFs(:,ii) = InvDynModel_3DOF(fe(:,ii),th,dth,ddth, 'IsFs', 0);
    tau_s(:,ii) = InvDynModel_3DOF(fe(:,ii),th,dth,ddth);
    tau_s2(:,ii) = InvDynModel_3DOF(fe(:,ii),th,dth,ddth, 'nu', ...
        [0.9488, 0.8345, 0.8345]);
    tau_s3(:,ii) = InvDynModel_3DOF(fe(:,ii),th,dth,ddth, 'nu', ...
        [0.9488, 0.7345, 0.8345]);    
    tau_s4(:,ii) = InvDynModel_3DOF(fe(:,ii),th,dth,ddth, 'nu', ...
        [0.9488, 0.6345, 0.8345]);
end
delete(f)
def_col = lines(8);

% direct dynamic modeling comparison
figure
tiledlayout(3,1,'TileSpacing','compact','Padding','compact')
for i = 1:3
    nexttile
    hold on
    plot(time, th_s(i,:), 'color', [def_col(1,:), 0.2])
    plot(time, th_m(i,:), 'color', [def_col(2,:), 0.2])
    p1 = plot(time, filtfilt(b,a,th_s(i,:)')', 'color', def_col(1,:));
    p0 = plot(time, filtfilt(b,a,th_m(i,:)')', 'color', def_col(2,:));  
    if i == 1   
        title('Direct dynamic model')
        legend([p1,p0], {'simulated', 'measured'})
    end
    ylabel('Angle (rad)')
end

% inverse dynamic comparison with meas.
figure
tiledlayout(3,1,'TileSpacing','compact','Padding','compact')
for i = 1:3
    nexttile
    hold on
    plot(time, tau_s(i,:), 'color', [def_col(2,:), 0.2])
    %plot(time, tau_s_noFs(i,:), 'color', [def_col(3,:), 0.2])
    plot(time, tau_m(:,i), 'color', [def_col(1,:), 0.2])
    p1 = plot(time, filtfilt(b,a,tau_s(i,:)')', 'color', def_col(2,:));
    %p3 = plot(time, filtfilt(b,a,tau_s_noFs(i,:)')', 'color', def_col(3,:));  
    p0 = plot(time, filtfilt(b,a,tau_m(:,i))', 'color', def_col(1,:));  
    if i == 1   
        title('Inverse dynamic model')
        %legend([p0,p1,p3], {'measured', 'simulated', 'sim wo Fs'})
        legend([p0,p1], {'measured', 'reconstructed'})
    end
    ylabel('torque (N.m)')
end

% reconstruction error
% inverse dynamic comparison with meas.
figure
tiledlayout(3,1,'TileSpacing','compact','Padding','compact')
for i = 1:3
    nexttile
    hold on
    plot(time, tau_m(:,i)' - tau_s(i,:), 'color', [def_col(1,:), 0.2])
    p0 = plot(time, filtfilt(b,a,tau_m(:,i)) - filtfilt(b,a,tau_s(i,:)'),...
        'color', def_col(1,:));  
    if i == 1   
        title('Inverse dynamic model')
        %legend([p0,p1,p3], {'measured', 'simulated', 'sim wo Fs'})
        legend(p0, "reconstr. err")
    end
    ylabel('torque (N.m)')
end

%
figure
tiledlayout(3,1,'TileSpacing','compact','Padding','compact')
for i = 1:3
    nexttile
    hold on
    plot(time, tau_m(:,i), 'color', [def_col(1,:), 0.2])
    plot(time, tau_s(i,:), 'color', [def_col(2,:), 0.2])
    plot(time, tau_s2(i,:), 'color', [def_col(3,:), 0.2])
    plot(time, tau_s3(i,:), 'color', [def_col(4,:), 0.2])
    plot(time, tau_s4(i,:), 'color', [def_col(5,:), 0.2])
    p0 = plot(time, filtfilt(b,a,tau_m(:,i))', 'color', def_col(1,:));  
    p1 = plot(time, filtfilt(b,a,tau_s(i,:)')', 'color', def_col(2,:));
    p2 = plot(time, filtfilt(b,a,tau_s2(i,:)')', 'color', def_col(3,:));
    p3 = plot(time, filtfilt(b,a,tau_s3(i,:)')', 'color', def_col(4,:));
    p4 = plot(time, filtfilt(b,a,tau_s4(i,:)')', 'color', def_col(5,:));
    if i == 1   
        title('Inverse dynamic model')
        %legend([p0,p1,p3], {'measured', 'simulated', 'sim wo Fs'})
        legend([p0,p1], {'measured', 'reconstructed'})
    end
    ylabel('torque (N.m)')
end

% % nu influence
% figure
% tiledlayout(3,1,'TileSpacing','compact','Padding','compact')
% for i = 1:3
%     nexttile
%     hold on
%     for j = 1:length(nu_list)
%         plot(time, tau_s_nu(i,:,j), 'color', [def_col(j+1,:), 0.2])
%         p1(j) = plot(time, filtfilt(b,a,tau_s_nu(i,:,j)')', 'color', def_col(j+1,:));
%     end
%     plot(time, tau_m(:,i), 'color', [def_col(1,:), 0.2])
%     p0 = plot(time, filtfilt(b,a,tau_m(:,i))', 'color', def_col(1,:));  
%     if i == 1   
%         title('Inverse dynamic model')
%         legend([p0,p1], convertStringsToChars(["measured", "nu="+string(nu_list)]))
%     end
%     ylabel('torque (N.m)')
% end