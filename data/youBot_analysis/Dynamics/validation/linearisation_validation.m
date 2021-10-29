% test linearisation youBot 
% (run youbot_dynamic_equation_linearisation.mlx, before running this)

load('dynamic_sim/youBot_linearisation_alone_tau_fz_inputs.mat')
load('dynamic_sim/youBot_linearisation_alone_tau_fz_inputs_discrete.mat')

Sigmaf = J0*minreal(tf(youBot_lin_alone(1:3, 1))); % dX/fz X = [x,z,theta]'
Sigma1 = J0*minreal(tf(youBot_lin_alone(1:3, 2:4))); % dX/tau

Sigmafz = Sigmaf(2,1); % dz/fz
Sigma1z = Sigma1(2,1:3); % dz/tau
Sigmafz.OutputName = 'dz';
Sigma1z.OutputName = 'dz';

youBot_model_test = parallel(Sigma1z,Sigmafz,'name');

% discrete
Sigmaf_d = J0*minreal(tf(youBot_lin_alone_discrete(1:3, 1))); % dX/fz X = [x,z,theta]'
Sigma1_d = J0*minreal(tf(youBot_lin_alone_discrete(1:3, 2:4))); % dX/tau

Sigmafz_d = Sigmaf_d(2,1); % dz/fz
Sigma1z_d = Sigma1_d(2,1:3); % dz/tau
Sigmafz_d.OutputName = 'dz';
Sigma1z_d.OutputName = 'dz';

youBot_model_test_d = parallel(Sigma1z_d,Sigmafz_d,'name');
youBot_model_test_d_tustin = c2d(youBot_model_test, dt, 'tustin');
youBot_model_test_d_zoh = c2d(youBot_model_test, dt, 'zoh');

tmp = J0*dyn_youBot_model_tf; % model from youbot_dynamic_equation_linearisation.mlx
dyn_youBot_model_tf_fz = tmp(2,:);

%% manual bode plots
freq = logspace(-2,log10(pi/dt),1e3);
for i = 1:length(youBot_model_test)
    [mag_smlk(:,i), pha_smlk(:,i)] = bode(youBot_model_test(1,i),freq);
    [mag_anly(:,i), pha_anly(:,i)] = bode(dyn_youBot_model_tf_fz(1,i),freq);
    [mag_smlk_d(:,i), pha_smlk_d(:,i)] = bode(youBot_model_test_d(1,i),freq);
    [mag_smlk_d_tustin(:,i), pha_smlk_d_tustin(:,i)] = bode(youBot_model_test_d_tustin(1,i),freq);
    [mag_smlk_d_zoh(:,i), pha_smlk_d_zoh(:,i)] = bode(youBot_model_test_d_zoh(1,i),freq);
end
var_names = ["tau_{r1}","tau_{r2}","tau_{r3}","f_z"];
color_list = lines(7);
figure('DefaultAxesFontSize',14)
for i = 1:4
    % magnitude
    subplot(2,4,i)
    pm1 = semilogx(freq, 20*log10(abs(mag_smlk(:,i))), 'Color', color_list(1,:));
    hold on
    pm2 = semilogx(freq, 20*log10(abs(mag_anly(:,i))), '--', 'Color', color_list(2,:));
    pm3 = semilogx(freq, 20*log10(abs(mag_smlk_d(:,i))), 'Color', color_list(3,:));
    pm4 = semilogx(freq, 20*log10(abs(mag_smlk_d_tustin(:,i))), '--', 'Color', color_list(4,:));   
    pm5 = semilogx(freq, 20*log10(abs(mag_smlk_d_zoh(:,i))), ':', 'Color', color_list(5,:));       
    title(var_names(i))
    % phase
    subplot(2,4,4+i)
    pp1 = semilogx(freq, mod(pha_smlk(:,i),-360), 'Color', color_list(1,:));
    hold on
    pp2 = semilogx(freq, mod(pha_anly(:,i),-360), '--', 'Color', color_list(2,:));
    pp3 = semilogx(freq, mod(pha_smlk_d(:,i),-360), 'Color', color_list(3,:));
    pp4 = semilogx(freq, mod(pha_smlk_d_tustin(:,i),-360), '--', 'Color', color_list(4,:));
    pp5 = semilogx(freq, mod(pha_smlk_d_zoh(:,i),-360), ':', 'Color', color_list(5,:));
end
subplot(2,4,1)
ylabel('Magnitude (dB)')
legend([pm1 pm2 pm3, pm4, pm5],'simulink','analytic','smlk. discr.','tustin','zoh')
subplot(2,4,5)
ylabel('Phase (deg)')
%ylim([-361 inf])
%yticks([-360, -180, 0, 180])
legend([pp1 pp2 pp3, pp4, pp5],'simulink','analytic','smlk. discr.','tustin','zoh')
xlabel('Angular frequency (rad.s^{-1})')