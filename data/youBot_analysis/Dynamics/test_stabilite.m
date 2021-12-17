% figure
% impulse(H_test)

K_test2 = 50;
B_test2 = 15;
M_test2 = 1;
mu2 = -(K_test2 + B_test2*s + M_test2*s^2)/s;

% Validation de la séparation du modèle du robot contrôlé
figure
bode(sigma_1*Ctrl + sigma_f)
hold on
bode(dyn_model_ss(2), 'r--')

% Effet de la force directement sur les transmissions
figure
bode(sigma_f)

% Effet du contrôle sans les transmissions et avec
figure
bode(sigma_1*Ctrl)
hold on
bode(sigma_1*Ctrl + sigma_f)
legend('\nu=0', '\nu~=0')

% Couplage du robot sans les transmissions avec un environnement
figure
bode(-mu*sigma_1*Ctrl)
figure
nyquist(-mu*sigma_1*Ctrl)
damp(-mu*sigma_1*Ctrl)

% filtre passe bas
tau_c = 0.1/(2*pi);
lwp_filt = 1/(1 + tau_c*s)^2;

% 
figure
bode(-lwp_filt*mu, -mu)
legend('\mu filt', '\mu ')

% 
figure
bode(-lwp_filt*mu, -lwp_filt*mu2, dyn_model_ss(2))
legend('\mu', '\mu_2', '\Sigma')

% Boucle fermée sans les transmissions
Sig_v_bf = feedback(sigma_1*Ctrl, -lwp_filt*mu);
zpk(Sig_v_bf)
figure
impulse(Sig_v_bf)

% Boucle fermée système total
Sig_bf = feedback(-lwp_filt*mu*dyn_model_ss(2), 1);
Sig_bf_v = feedback(dyn_model_ss(2), -lwp_filt*mu);
Sig_bf2 = feedback(dyn_model_ss(2), -lwp_filt*mu2);
zpk(Sig_bf)
zpk(Sig_bf2)
figure
impulse(Sig_bf,Sig_bf2)
figure
step(Sig_bf,Sig_bf2)
damp(Sig_bf2)

figure
step(Sig_bf_v)
yyaxis right
hold on
step(Sig_bf)

% Boucle fermée avec les transmissions seules vis à vis d'une perturbation
% en force
Sig_f_bf = feedback(sigma_f, -mu);
damp(Sig_f_bf)
figure
impulse(Sig_f_bf)

% sum
figure
impulse(Sig_v_bf + Sig_f_bf)
hold on
impulse(H_test)


Ctrl_i = cz0(1) + cz0(2)/s;
Hbo_test_sep1 = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));
Ctrl_i = 10*cz0(1) + cz0(2)/s;
Hbo_test_sep2 = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));

figure
nyquist(Hbo_test_sep1,Hbo_test_sep2)
figure
nichols(Hbo_test_sep1,Hbo_test_sep2)

%% Filter time constant influence
fc_list = [1;5;10;15;20];
figure(100)
figure(101)
for i = 1:length(fc_list)
    fc = fc_list(i);
    tau_m = 1/(fc*2*pi);
    mu = -(K_test + B_test*s + M_test*s^2)/s*1/(1+tau_m*s)^2;
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl));
    Hbo(i) = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));
    figure(100)
    nichols(Hbo(i))
    hold on
    figure(101)
    step(Hbf(i))
    hold on
end
figure(101)
ylim([-1, 1])
