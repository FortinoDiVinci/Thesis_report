%% run youbot_dynamic_equation_linearisation.mlx first
addpath '..\..\utils'
% figure
% impulse(H_test)

% K_test2 = 163.3;
% B_test2 = 10.15;
% M_test2 = 0.413;
s = tf('s');

% K_test2 = 362.7;
% B_test2 = 9.9;
% M_test2 = 0.567;
% K_test2 = 479;
% B_test2 = 11.5;
% M_test2 = 0.626;
K_test2 = 363;
B_test2 = 9.9;
M_test2 = 0.567;

mu2 = -(K_test2 + B_test2*s + M_test2*s^2)/s;

color_Orient = [0,78,125]/255;
color_Lavender = [185,177,215]/255;
color_Shiraz = [198,11,70]/255;
color_Prune = [99, 0, 60]/255;
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
bode(-lwp_filt*mu2, -mu2)
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

colors_stable = [linspace(color_Orient(1),color_Lavender(1),6)', ...
    linspace(color_Orient(2),color_Lavender(2),6)', ....
    linspace(color_Orient(3),color_Lavender(3),6)'];
colors_not_stable = [linspace(color_Lavender(1),color_Shiraz(1),4)', ...
    linspace(color_Lavender(2),color_Shiraz(2),4)', ....
    linspace(color_Lavender(3),color_Shiraz(3),4)'];

%% Filter time constant influence
%fc_list = linspace(1.3250, 1.4016,9)
fc_list = [5,10,15,20,30,40,50,100,1000];
Ctrl_i = cz0(1) + cz0(2)/s;
for i = 1:length(fc_list)
    fc = fc_list(i);
    tau_m = 1/(fc*2*pi);
    mu = -(K_test2 + B_test2*s + M_test2*s^2)/s/(1+tau_m*s)^2;
    % Hbf(i) = minreal((1 -
    % mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl)); % erroneous ...
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i)); 
    Hbo(i) = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));
    figure(100)
    nichols(Hbo(i), 'b')
    hold on
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    if i < 7
        set(lineHandle,'Color',colors_stable(i,:));
    else
        set(lineHandle,'Color',colors_not_stable(i-5,:));
    end
%     figure(101)
%     step(Hbf(i))
%     hold on
%     figure(102)
%     nyquist(Hbo(i))
%     hold on
%     figure(103)
%     bode(Hbo(i), 'b')
%     hold on
%     lineHandle = findobj(gcf,'Type','line','-and','Color','b');
%     if i < 7
%         set(lineHandle,'Color',colors_stable(i,:));
%     else
%         set(lineHandle,'Color',colors_not_stable(i-5,:));
%     end
end
%figure(101)
%ylim([-1, 1])

%% Admittance control gain influence
% param = [min_with_confidence, median, max_with confidence]
% K_000 = [163.5-52, 151, 194.3+29.9]; % user000
% B_000 = [10.15-2.14, 15.72, 15.72+0.9];
% M_000 = [0.351-0.012, 0.351, 0.413+0.035];
% K_000 = [167-42, 196, 252+63]; % user000
% B_000 = [15-2.8, 12.9, 19.8+2.4];
% M_000 = [0.402-0.043, 0.370, 0.529+0.048];
K_000 = [151, 194, 164]; % user000
B_000 = [15, 12.9, 10.1];
M_000 = [0.349, 0.374, 0.413];
tau_m = 1/(10*2*pi);

colors_1 = [linspace(color_Orient(1),color_Lavender(1),5)', ...
    linspace(color_Orient(2),color_Lavender(2),5)', ....
    linspace(color_Orient(3),color_Lavender(3),5)'];
colors_2 = [linspace(color_Lavender(1),color_Shiraz(1),6)', ...
    linspace(color_Lavender(2),color_Shiraz(2),6)', ....
    linspace(color_Lavender(3),color_Shiraz(3),6)'];
colors_2(1,:) = [];
colors_plot = [colors_1;colors_2];

mu = -(K_000(2) + B_000(2)*s + M_000(2)*s^2)/s/(1+tau_m*s)^2;
% Ki influence on close loop
kp = 0.015;
ki_list = [0.01, 0.04, 0.08, 0.15, 0.3, 0.6, 1.2, 2.4, 4.8, 10.5];
%ki_list = linspace(0.01, 4.8, 30);
t_data = linspace(0,4,600);
u_data = 5*sin(2*pi*0.9.*t_data); % sinusoidal input
clear y
for i = 1:length(ki_list)
    ctrl_i = kp + ki_list(i)/s;
    Hbf = minreal((1 - mu*(sigma_f+sigma_1*ctrl_i))\(sigma_f+sigma_1*ctrl_i));
    Hbo = minreal(-mu*(sigma_f+sigma_1*ctrl_i));
    isStab(i) = isstable(Hbf);
%     figure(90)
%     [y_out,t_out] = lsim(Hbf/s,u_data,t_data);
%     if i < 6
%         plot(t_out, y_out, 'Color', colors_1(i,:))
%     else
%         plot(t_out, y_out, 'Color', colors_2(i-5,:))
%     end
%     hold on
%     y(:,i) = 1e2.*y_out; %m->mm
    figure(92)
    bode(minreal(Hbf), 'b')
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',colors_plot(i,:));
    hold on
    figure(93)
    bode(minreal(minreal(sigma_f+sigma_1*ctrl_i)), 'b')
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',colors_plot(i,:));
    hold on
    figure(94)
    nyquist(minreal(Hbo), 'b')
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',colors_plot(i,:));
    hold on
end

% K_t = 50;
% B_t = 5;
% M_t = 10;
% ctrl_i = kp + ki_list(3)/s;
% figure
% bode(1/(M_t*s^2))
% hold on
% bode(minreal((sigma_f+sigma_1*ctrl_i)/s))
% bode(minreal((sigma_f+sigma_1*(kp + ki_list(6)/s))/s))
% bode(minreal( (sigma_f )/s ))
% legend('Masse pure', "Ki="+string(ki_list(3)), "Ki="+string(ki_list(6)), 'HBO_rob')

figure
bode(minreal(sigma_f/s))
hold on
bode(minreal( (sigma_f + sigma_1*0)/s ))

figure
bode(1/(M_t*s))
hold on
bode(minreal((sigma_f+sigma_1*ctrl_i)))
legend('Masse pure', 'HBO_rob')

%[y_out,t_out] = lsim(Hbf/s,u_data,t_data);
figure(90)
yyaxis right 
plot(t_out, u_data, '--')
grid on

% u_in = u_data';
% t = t_data';
% table_param = table(t, u_in, y);
% write(table_param,'ki_influence.csv','Delimiter',',');

% Kp influence on close loop
kp_list = [0.001, 0.002, 0.0035, 0.007, 0.015, 0.03, 0.06, 0.09, 0.12, 0.245];
ki = 0.08;
t_data = linspace(0,4,1000);
u_data = 5*sin(2*pi*0.9.*t_data); % sinusoidal input
clear y
for i = 1:10
    ctrl_i = kp_list(i) + ki/s;
    Hbf = minreal((1 - mu*(sigma_f+sigma_1*ctrl_i))\(sigma_f+sigma_1*ctrl_i));
    Hbo = minreal(-mu*(sigma_f+sigma_1*ctrl_i));
%     figure(50)
%     [y_out,t_out] = lsim(Hbf/s,u_data,t_data);
%     if i < 6
%         plot(t_out, y_out, 'Color', colors_1(i,:))
%     else
%         plot(t_out, y_out, 'Color', colors_2(i-5,:))
%     end
%     hold on
%     y(:,i) = 1e2.*y_out; %m->mm
    figure(51)
    bode(minreal(sigma_f+sigma_1*ctrl_i),'b')
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',colors_plot(i,:));
    hold on
    figure(52)
    bode(minreal(Hbf),'b')
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',colors_plot(i,:));
    hold on
    figure(53)
    nyquist(minreal(Hbo),'b')
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',colors_plot(i,:));
    hold on
end
[y_out,t_out] = lsim(Hbf/s,u_data,t_data);
figure(91)
yyaxis right 
plot(t_out, u_data, '--')

u_in = u_data';
t = t_data';
table_param = table(t, u_in, y);
write(table_param,'kp_influence.csv','Delimiter',',');

% stability mapping
fb = 10;
tau_m = 1/(fb*2*pi);
% kp_list = linspace(5e-3,0.25,40);
% ki_list = linspace(5e-2,3,40);
kp_list = linspace(0,0.15,50);
ki_list = linspace(0,0.8,50);

mu = -(K_000(2) + B_000(2)*s + M_000(2)*s^2)/s/(1+tau_m*s)^2;
for i = 1:length(kp_list)
    for j = 1:length(ki_list)
        ctrl_i = kp_list(i) + ki_list(j)/s;
        Hbf = minreal((1 - mu*(sigma_f+sigma_1*ctrl_i))\(sigma_f+sigma_1*ctrl_i));
%         Hbo = minreal(-mu*(sigma_f+sigma_1*ctrl_i));
        stab_map(i,j) = isstable(Hbf);
        %[gain_margin_map(i,j),phase_margin_map(i,j)] = margin(Hbo);
        %a = stepinfo(Hbf);
        %peak_time(i,j) = a.PeakTime;
        %set_time(i,j) = a.SettlingTime;        
%         if(stab_map(i,j) == 0)
%             vector_margin(i,j) = 0;
%         else
%             [H,~] = freqresp(Hbo);
%             vector_margin(i,j) = min(abs(1+squeeze(H)));
%         end
        [~,zeta] = damp(Hbf);
        damp_ratio(i,j) = min(zeta);
    end
end

mu = -(3*K_000(2) + B_000(2)*s + M_000(2)*s^2)/s/(1+tau_m*s)^2;
for i = 1:length(kp_list)
    for j = 1:length(ki_list)
        ctrl_i = kp_list(i) + ki_list(j)/s;
        Hbf = minreal((1 - mu*(sigma_f+sigma_1*ctrl_i))\(sigma_f+sigma_1*ctrl_i));
%         Hbo = minreal(-mu*(sigma_f+sigma_1*ctrl_i));
        stab_map_hstiff(i,j) = isstable(Hbf);
        %[gain_margin_map_hstiff(i,j),phase_margin_map_hstiff(i,j)] = margin(Hbo);
        %a = stepinfo(Hbf);
        %peak_time_hstiff(i,j) = a.PeakTime;
        %set_time_hstiff(i,j) = a.SettlingTime;
%         [H,~] = freqresp(Hbo);
%         if(stab_map_hstiff(i,j) == 0)
%             vector_margin_hstiff(i,j) = 0;
%         else
%             [H,~] = freqresp(Hbo);
%             vector_margin_hstiff(i,j) = min(abs(1+squeeze(H)));
%         end
        [~,zeta] = damp(Hbf);
        damp_ratio_hstiff(i,j) = min(zeta);
    end
end

figure
subplot(2,1,1)
surf(ki_list, kp_list, double(stab_map))
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
title("f_c=" + string(fb))
subplot(2,1,2)
surf(ki_list, kp_list, double(stab_map_hstiff))
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")

figure
subplot(2,1,1)
surf(ki_list, kp_list, damp_ratio)
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
subplot(2,1,2)
surf(ki_list, kp_list, damp_ratio_hstiff)
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")

figure
subplot(2,1,1)
surf(ki_list, kp_list, vector_margin)
view(2)
colorbar
subplot(2,1,2)
surf(ki_list, kp_list, vector_margin_hstiff)
view(2)
colorbar

%ki_tmp = repmat(ki_list, length(kp_list(1:end-2)), 1);
%kp = repmat(kp_list(1:end-2)', length(ki_list), 1);
ki_tmp = repmat(ki_list, length(kp_list), 1);
ki = ki_tmp(:);
kp = repmat(kp_list', length(ki_list), 1);

zeta = damp_ratio(:);
tab_zeta = table(kp, ki, zeta);
write(tab_zeta,'damping_map_nominal_theory.csv','Delimiter',',');
%
zeta = damp_ratio_hstiff(:);
tab_zeta = table(kp, ki, zeta);
write(tab_zeta,'damping_map_stiff_theory.csv','Delimiter',',');

% DG_tmp = vector_margin(1:end-2,:);
% DG = DG_tmp(:);
% tab_1 = table(kp, ki, DG);
% write(tab_1,'vector_margin_map_nominal_theory.csv','Delimiter',',');
% %
% DG_tmp = vector_margin_hstiff(1:end-2,:);
% DG = DG_tmp(:);
% tab_1 = table(kp, ki, DG);
% write(tab_1,'vector_margin_map_stiff_theory.csv','Delimiter',',');
% %
% Dphi_tmp = phase_margin_map(1:end-2,:);
% Dphi = Dphi_tmp(:);
% tab_1 = table(kp, ki, Dphi);
% write(tab_1,'stability_map_nominal_theory.csv','Delimiter',',');
% %
% Dphi_tmp = phase_margin_map_hstiff(1:end-2,:);
% Dphi = Dphi_tmp(:);
% tab_1 = table(kp, ki, Dphi);
% write(tab_1,'stability_map_stiff_theory.csv','Delimiter',',');

figure
% nominal stiff
subplot(2,3,1)
surf(ki_list, kp_list(1:end-2), phase_margin_map(1:end-2,:))
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
title("\Delta_\phi nom")
subplot(2,3,2)
surf(ki_list, kp_list(1:end-2), vector_margin(1:end-2,:))
view(2)
colorbar
xlabel("Ki")
title("\Delta_M nom")
subplot(2,3,3)
surf(ki_list, kp_list(1:end-2), gain_margin_map(1:end-2,:))
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
title("K_G nom")
% surf(ki_list, kp_list(1:end-2), set_time(1:end-2,:))
% view(2)
% colorbar
% xlabel("Ki")
% title("Set time nom")
% x3 stiff
subplot(2,3,4)
surf(ki_list, kp_list(1:end-2), phase_margin_map_hstiff(1:end-2,:))
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
title("\Delta_\phi K=453")
subplot(2,3,5)
surf(ki_list, kp_list(1:end-2), vector_margin_hstiff(1:end-2,:))
view(2)
colorbar
xlabel("Ki")
title("\Delta_M K=453")
subplot(2,3,6)
surf(ki_list, kp_list(1:end-2), gain_margin_map_hstiff(1:end-2,:))
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
title("K_G K=453")
% surf(ki_list, kp_list(1:end-2), set_time_hstiff(1:end-2,:))
% view(2)
% colorbar
% xlabel("Ki")
% title("Set time K=453")

% stability mapping to fit experimental data
kp_list = linspace(0,4.5e-2,37);
ki_list = linspace(0,1,40);
tau_m = 1/(25*2*pi); %fc = 25hz
stiff_Gain = 4;
mu = -(stiff_Gain*K_000(2) + B_000(2)*s + M_000(2)*s^2)/s/(1+tau_m*s)^2;
clear vector_margin phase_margin_map_hstiff2 stab_map_hstiff2 vector_margin_hstiff2
for i = 1:length(kp_list)
    for j = 1:length(ki_list)
        ctrl_i = kp_list(i) + ki_list(j)/s;
        Hbf = minreal((1 - mu*(sigma_f+sigma_1*ctrl_i))\(sigma_f+sigma_1*ctrl_i));
        Hbo = minreal(-mu*(sigma_f+sigma_1*ctrl_i));
        stab_map_hstiff2(i,j) = isstable(Hbf);
        [~,phase_margin_map_hstiff2(i,j)] = margin(Hbo);
        [H,~] = freqresp(Hbo);
        vector_margin_hstiff2(i,j) = min(abs(1+squeeze(H))); 
        %min(sqrt( (real(squeeze(H)) + 1).^2 + (imag(squeeze(H)) + 0).^2 ));
    end
end

figure
% cocontraction stiffness
surf(ki_list, kp_list, phase_margin_map_hstiff2)
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
title("\Delta_\phi K="+string(4*K_000(2)))

figure
% cocontraction stiffness
surf(ki_list, kp_list, vector_margin_hstiff2)
view(2)
colorbar
ylabel("Kp")
xlabel("Ki")
title("\Delta_M K="+string(stiff_Gain*K_000(2)))


%% environnement variations
Ctrl_i = cz0(1) + cz0(2)/s;
tau_m = 1/(10*2*pi);
M_data = repmat(0.4,10,1);
B_data = repmat(11.5,10,1);
K_data = repmat(200,10,1);
%K_data = linspace(50,4000,10);
K_data = [25,50,100,200,400,800,1400,2200,3500,4700];
%B_data = linspace(4,17,10);
%M_data = linspace(0.2,0.8,10);
colors_1 = [linspace(color_Orient(1),color_Lavender(1),5)', ...
    linspace(color_Orient(2),color_Lavender(2),5)', ....
    linspace(color_Orient(3),color_Lavender(3),5)'];
colors_2 = [linspace(color_Lavender(1),color_Shiraz(1),6)', ...
    linspace(color_Lavender(2),color_Shiraz(2),6)', ....
    linspace(color_Lavender(3),color_Shiraz(3),6)'];
colors_2(1,:) = [];
t_data = linspace(0,4,400);
u_data = 5*sin(2*pi*0.9.*t_data); % sinusoidal input
clear y
for i = 1:10
    mu = -(K_data(i) + B_data(i)*s + M_data(i)*s^2)/s/(1+tau_m*s)^2;
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i));
    %Hbf_f(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(- mu*(sigma_f+sigma_1*Ctrl_i)));
    Hbo(i) = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));
%     figure(110)
%     nichols(Hbo(i), 'b')
%     hold on
%     lineHandle = findobj(gcf,'Type','line','-and','Color','b');
%     if i < 6
%         set(lineHandle,'Color',colors_1(i,:));
%     else
%         set(lineHandle,'Color',colors_2(i-5,:));
%     end
%     figure(111)
% %     [yf_out,t_out] = lsim(Hbf_f(i)/s,u_data,t_data);
%     [y_out,t_out] = lsim(Hbf(i)/s,u_data,t_data);
%     if i < 6
%         plot(t_out, y_out, 'Color', colors_1(i,:))
%     else
%         plot(t_out, y_out, 'Color', colors_2(i-5,:))
%     end
%     hold on
%     figure(112)
%     if i < 7
%         plot(t_out, yf_out, 'Color', colors_stable(i,:))
%     else
%         plot(t_out, yf_out, 'Color', colors_not_stable(i-5,:))
%     end
%     hold on
%     y(:,i) = 1e2.*y_out; %m->cm
    figure(113)
    bode(Hbo(i), 'b')
    hold on
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    if i < 6
        set(lineHandle,'Color',colors_1(i,:));
    else
        set(lineHandle,'Color',colors_2(i-5,:));
    end
    figure(114)
    bode(minreal(Hbf(i)/s*100), 'b') %m->cm
    hold on
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    if i < 6
        set(lineHandle,'Color',colors_1(i,:));
    else
        set(lineHandle,'Color',colors_2(i-5,:));
    end
end
figure(111)
yyaxis right 
plot(t_out, u_data, '--')
u_in = u_data';
t = t_data';
table_param = table(t, u_in, y);
% write(table_param,'environment_stiffness_influence.csv','Delimiter',',');
% write(table_param,'environment_damping_influence.csv','Delimiter',',');
% write(table_param,'environment_mass_influence.csv','Delimiter',',');

%% phase margin against stiffness
%linspace(50,3000,119);
K_data = logspace(0.3,3.7);
%B_data = 9.9;
B_data = 11.5;
%M_data = 0.567;
M_data = 0.4;
%colors_map = jet(length(K_data));
colors_m1 = [linspace(color_Orient(1),color_Lavender(1),25)', ...
            linspace(color_Orient(2),color_Lavender(2),25)', ....
            linspace(color_Orient(3),color_Lavender(3),25)'];
colors_m2 = [linspace(color_Lavender(1),color_Shiraz(1),26)', ...
            linspace(color_Lavender(2),color_Shiraz(2),26)', ....
            linspace(color_Lavender(3),color_Shiraz(3),26)'];
colors_m2(1,:) = [];
colors_map = [colors_m1; colors_m2];

for i = 1:length(K_data)
    mu = -(K_data(i) + B_data*s + M_data*s^2)/s/(1+tau_m*s)^2;
%    Hbo(i) = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i));
    
%    [Gm(i),Pm(i)] = margin(Hbo(i));
%     figure(120)
%     [y_out,t_out] = lsim(Hbf(i)/s,u_data,t_data);
%     plot(t_out, y_out)
%     hold on
%     [freq,zeta] = damp(Hbf(i));
%     oscil_idx = zeta<0.999;
%     damp_score_stiff{i} = [freq(oscil_idx)./(2*pi),zeta(oscil_idx)];
    figure(121)
    pzmap(Hbf(i),'b')
    markerHandle = findobj(gcf,'Color','b');
    set(markerHandle,'Color',colors_map(i,:));   
    hold on    
end
K_data = 1:1:5001;
for i = 1:length(K_data)
    mu = -(K_data(i) + B_data*s + M_data*s^2)/s/(1+tau_m*s)^2;
    Hbo(i) = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i));
    [H,~] = freqresp(Hbo(i));
    vector_margin_k(i) = min(abs(1+squeeze(H))); 
    stiff_stab(i) = isstable(Hbf(i));
    if(stiff_stab(i) == 0)
        vector_margin_k(i) = 0;
    end
end
figure
plot(K_data, vector_margin_k)
K=(1:5001)'./1e3;
delta_M = vector_margin_k';
% table_d_mod = table(K, delta_M);
% write(table_d_mod,'environment_stiffness_vector_margin.csv','Delimiter',',');

figure
plot(1:5001, Pm)
grid on
D_phi = Pm';
K=(1:5001)'./1e3;
% table_d_phi = table(K, D_phi);
% write(table_d_phi,'environment_stiffness_phase_margin.csv','Delimiter',',');

dPm = Iu_diffcent(D_phi,(1:5001)');
figure
plot(1:5001,dPm)
grid on

% damping ratio plot
K_data = linspace(50,3000,119);
B_data = 9.9;
M_data = 0.567;
for i = 1:length(K_data)
    mu = -(K_data(i) + B_data*s + M_data*s^2)/s/(1+tau_m*s)^2;
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i));
    %stiff_stab(i) = isstable(Hbf(i));
%     figure(120)
%     [y_out,t_out] = lsim(Hbf(i)/s,u_data,t_data);
%     plot(t_out, y_out)
%     hold on
    [freq,zeta] = damp(Hbf(i));
    oscil_idx = zeta<0.999;
    damp_score_stiff{i} = [freq(oscil_idx)./(2*pi),zeta(oscil_idx)];
end
nb_bad_damp = cellfun(@(x) size(x,1), damp_score_stiff);

colors_grad = [linspace(color_Orient(1),color_Lavender(1),ceil(length(K_data)/2))', ...
               linspace(color_Orient(2),color_Lavender(2),ceil(length(K_data)/2))', ....
               linspace(color_Orient(3),color_Lavender(3),ceil(length(K_data)/2))'];
colors_grad_2 = [linspace(color_Lavender(1),color_Shiraz(1),ceil(length(K_data)/2)+1)', ...
               linspace(color_Lavender(2),color_Shiraz(2),ceil(length(K_data)/2)+1)', ....
               linspace(color_Lavender(3),color_Shiraz(3),ceil(length(K_data)/2)+1)'];
colors_grad = [colors_grad; colors_grad_2(2:end,:)];
figure
hold on
for i=1:length(K_data)
    plot(damp_score_stiff{i}(:,1), damp_score_stiff{i}(:,2), '*', 'Color', colors_grad(i,:))
end

%% phase margin against damping
K_data = 362.7;
M_data = 0.567;
B_data = linspace(2,100,99);
for i = 1:length(B_data)
    mu = -(K_data + B_data(i)*s + M_data*s^2)/s/(1+tau_m*s)^2;
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i));
    [freq,zeta] = damp(Hbf(i));
    oscil_idx = zeta<0.999;
    damp_score_damp{i} = [freq(oscil_idx)./(2*pi),zeta(oscil_idx)];
end

% damping ratio plot
colors_grad = [linspace(color_Orient(1),color_Shiraz(1),length(B_data))', ...
               linspace(color_Orient(2),color_Shiraz(2),length(B_data))', ....
               linspace(color_Orient(3),color_Shiraz(3),length(B_data))'];
figure
hold on
for i=1:length(B_data)
    plot(damp_score_damp{i}(:,1), damp_score_damp{i}(:,2), '*', 'Color', colors_grad(i,:))
end

%% phase margin against mass
K_data = 362.7;
B_data = 9.9;
M_data = linspace(0.1,10,100);
for i = 1:length(M_data)
    mu = -(K_data + B_data*s + M_data(i)*s^2)/s/(1+tau_m*s)^2;
    Hbf(i) = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i));
    [freq,zeta] = damp(Hbf(i));
    oscil_idx = zeta<0.999;
    damp_score_mass{i} = [freq(oscil_idx)./(2*pi),zeta(oscil_idx)];
end

% damping ratio plot
colors_grad = [linspace(color_Orient(1),color_Shiraz(1),length(M_data))', ...
               linspace(color_Orient(2),color_Shiraz(2),length(M_data))', ....
               linspace(color_Orient(3),color_Shiraz(3),length(M_data))'];
figure
hold on
for i=1:length(M_data)
    plot(damp_score_mass{i}(:,1), damp_score_mass{i}(:,2), '*', 'Color', colors_grad(i,:))
end

%% stability for all environment config (from measurements)
Ctrl_i = cz0(1) + cz0(2)/s;
tau_m = 1/(10*2*pi);
K_data = linspace(50,500,19);
B_data = linspace(3,17,15);
M_data = linspace(0.2,0.8,13);
t_data = linspace(0,3,300);
u_data = 5*sin(2*pi*0.9.*t_data); % sinusoidal input
for ki = 1:length(K_data)
    for bi = 1:length(B_data)
        for mi = 1:length(M_data)
            mu = -(K_data(ki) + B_data(bi)*s + M_data(mi)*s^2)/s/(1+tau_m*s)^2;
            Hbf = minreal((1 - mu*(sigma_f+sigma_1*Ctrl_i))\(sigma_f+sigma_1*Ctrl_i));
            Hbo = minreal(-mu*(sigma_f+sigma_1*Ctrl_i));
            stability_map(ki,bi,mi) = isstable(Hbf);
        end
    end
end