% fz needs to be computed
% only test for data_vfo_10.mat

%% TODO: This script needs to be cleaned to work as a stand alone prog

[b2,a2] = butter(10,9/(1/(2*dt)),'low'); 
fz2{exp_nb} = -1*filtfilt(b2,a2,fz_temp);

figure
hold on
plot(t{exp_nb}, -fz_temp)
plot(t{exp_nb}, fz{exp_nb})
plot(t{exp_nb}, fz2{exp_nb})
plot(t{exp_nb}(idx_perts), fz{exp_nb}(idx_perts), 'kp')
legend('original', '25Hz LP', '10Hz LP', 'pert')
xlim([52.5,54.5])

%[n,Wn] = buttord([8 11]/(1/(2*dt)),[6.5 12.5]/(1/(2*dt)),0.1,20);
[b3,a3] = ellip(5,0.1,60,7./(1/(2*dt)),'low');  
fz3{exp_nb} = -1*filtfilt(b3,a3,fz_temp);
figure
plot(t{exp_nb}, -fz_temp)
hold on
plot(t{exp_nb}, fz{exp_nb})
plot(t{exp_nb},fz3{exp_nb})
plot(t{exp_nb}(idx_perts), fz{exp_nb}(idx_perts), 'kp')
legend('original', '25Hz LP', 'custom', 'pert')
xlim([52.5,54.5])