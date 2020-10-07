
% i = 3;
% idx_st = 2.1e4;
% idx_end = 1.1e5; 
% 
% i = 7;
% idx_st = 4.2e4;
% idx_end = 1.6e5; 
%
% i = 10;
% idx_st = 1.45e4;
% idx_end = 1.1e5; 
i = 1; % martin
idx_st = 1.42e4;
idx_end = 3.2e5; 

%% ball signal processing

fc = 25; % cut off frequency
[b,a] = butter(2,fc/(1/(2*dt)),'low'); 

zb = filtfilt(b,a,z_b{i});
dzb = Iu_diffcent(zb,t{i});
zb_old = zb;
zb = zb(idx_st:idx_end);
dzb = dzb(idx_st:idx_end);

impact = crossing(dzb);
idx_imp = impact(diff(impact)>100);
idx_imp_inf = idx_imp(zb(idx_imp) < 0.5);

t_new = t{i}(idx_st:idx_end);

figure
plot(t{i}, zb_old)
hold on
plot(t_new, zb)
plot(t_new(idx_imp_inf), zb(idx_imp_inf), '*k')

%% disturbance data processing

t_dist_on = t_dist{i}(1:2:end);
t_dist_off = t_dist{i}(2:2:end);
t_dist_avg = mean([t_dist_on, t_dist_off], 2);

pert_idx = [];
for dist_idx = 1:length(t_dist_avg)
    if t_dist_avg(dist_idx) < min(t_new)
        continue;
    end
    pert_idx = [pert_idx, find(t_new >= t_dist_avg(dist_idx) , 1, 'first')];   
end

%% hand position signal processing

zh = filtfilt(b,a,z{i});
dzh = Iu_diffcent(zh,t{i});
zh = zh(idx_st:idx_end)*1e2; % conversion to cm
dzh = dzh(idx_st:idx_end);

figure
plot(zh-mean(zh), dzh)
hold on
plot(zh(idx_imp_inf)-mean(zh), dzh(idx_imp_inf), '.r', 'MarkerSize',30)
plot(zh(pert_idx)-mean(zh), dzh(pert_idx), '.m', 'MarkerSize',30)
xlim([-8, 8])
