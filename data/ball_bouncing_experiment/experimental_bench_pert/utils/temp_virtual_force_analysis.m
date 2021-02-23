%% impedance_estimation_list needs to be run before this script

delta_z = DIFF_TRAJECT(200, 200, ...
            z{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
delta_fz = DIFF_TRAJECT(200, 58, ...
            fz{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
delta_fz_ref = DIFF_TRAJECT(200, 200, ...
            fz{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
        
delta_z.computeDiffTraject('VirtTrajMethod', 'spline');
delta_fz.computeDiffTraject('VirtTrajMethod','spline','DiffDirection','neg');
delta_fz_ref.computeDiffTraject('VirtTrajMethod','spline','DiffDirection','neg');
delta_z.computeDerivatives(); 

impedance = IMPEDANCE_DATA(nb_param, length(dist_timings), idx_window_imp_eval);
impedance.init_phi(delta_z.diff_traject(1:idx_window_imp_eval,:), ...
    delta_z.d_diff_traject(1:idx_window_imp_eval,:), ... % speed
    delta_z.dd_diff_traject(1:idx_window_imp_eval,:)); % acceleration
impedance.init_y(delta_fz.diff_traject(1:idx_window_imp_eval,:));
impedance.lsq(); % least square optimization evaluation

% ref
impedance_ref = IMPEDANCE_DATA(nb_param, length(dist_timings), idx_window_imp_eval(28));
impedance_ref.init_phi(delta_z.diff_traject(1:idx_window_imp_eval(28),:), ...
    delta_z.d_diff_traject(1:idx_window_imp_eval(28),:), ... % speed
    delta_z.dd_diff_traject(1:idx_window_imp_eval(28),:)); % acceleration
impedance_ref.init_y(delta_fz_ref.diff_traject(1:idx_window_imp_eval(28),:));
impedance_ref.lsq();

figure
p0 = plot(t{exp_nb}, delta_fz.complete_traject);
hold on
for it = 1:size(delta_fz.t_traject, 2)
    p1 = plot(delta_fz.t_traject(:,it), delta_fz.virt_traject(:,it), 'Color', [0.8500, 0.3250, 0.0980]);
    p2 = plot(delta_fz_ref.t_traject(:,it), delta_fz_ref.virt_traject(:,it), 'Color', [0.4660, 0.6740, 0.1880]);
end
p3 = plot(t{exp_nb}(idx_perts), delta_fz.complete_traject(idx_perts), 'pk', 'MarkerSize', 15);
legend([p0, p1(1), p2(1), p3(1)], {'f_z', 'splines 58ms', 'splines 200ms', 'perturbations'});
xlabel('Time (s)')
ylabel('Force (N)')
title('Virtual force reconstruction comparison')

figure
subplot(3,1,1)
hold on
plot(impedance.r_2)
plot(impedance_ref.r_2)
legend('65ms splines','200ms splines')
title('Force reconstruction R^2 scores')
subplot(3,1,2)
hold on
plot(impedance.xi(1,:))
plot(impedance_ref.xi(1,:))
legend('65ms splines','200ms splines')
title('Stiffness')
subplot(3,1,3)
hold on
plot(impedance.xi(2,:))
plot(impedance_ref.xi(2,:))
yyaxis right
plot(impedance.xi(3,:))
plot(impedance_ref.xi(3,:))
legend('D 65ms','D 200ms','M 65ms','M 200ms')
title('Damping and Mass')