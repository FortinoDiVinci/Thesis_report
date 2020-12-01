clear all
close all

load('saved_pred.mat')


y = [];
y_hat_rnn = [];
y_hat_spl = [];
t_subsamp = (1:150)*1e-2;
t_ = (1:0.1:150)*1e-2;
err_rnn = [];
err_spl = [];

for data = Data 
    y(:,end+1) = interp1(t_subsamp,data{1}.y*Divisions.force + Offsets.force,t_);
    y_hat_rnn(:,end+1) = interp1(t_subsamp,data{1}.y_hat*Divisions.force + Offsets.force,t_); 
    err_rnn(:,end+1) = y(end-199:end,end) - y_hat_rnn(end-199:end,end);  
end

% spline prediction of the 200 last samples
t_s = [t_(end-201),t_(end-200),t_(end-1),t_(end)];
for y_i = y
    y_si = [y_i(end-201),y_i(end-200),y_i(end-1),y_i(end)];
    y_s = interp1(t_s, y_si, t_(end-200:end), "spline");
    y_s = y_s(2:end); % starting point is not included, but landing point is
    
    y_hat_spl(:,end+1) = y_s;
    err_spl(:,end+1) = y_i(end-199:end,end) - y_hat_spl(:,end);
end

figure('DefaultAxesFontSize',14)
histogram(err_rnn(:))
hold on
histogram(err_spl(:))
xlabel('Error (N)')
legend('RNN', 'Spline')

disp('Mean and std RNN')
std(err_rnn(:))
mean(err_rnn(:))
disp('Mean and std Splines')
std(err_spl(:))
mean(err_spl(:))

figure('DefaultAxesFontSize',14)
p1 = plot(err_rnn, 'b');
hold on
p2 = plot(err_spl, 'r');
for j=1:length(p1)
    p1(j).Color(4) = 0.3;
    p2(j).Color(4) = 0.3;
end
legend('RNN', 'Splines')
title('Errors')
xlabel('Time (ms)')
ylabel('Force (N)')

err = [];
rand_idxs = randperm(size(y,2));
figure('DefaultAxesFontSize',14)
for it = 1:9
    subplot(3,3,it)
    idx = rand_idxs(it);
    plot(t_, y(:,idx))
    hold on
    plot(t_(end-199:end), y_hat_rnn(end-199:end,idx))
    plot(t_(end-199:end), y_hat_spl(:,idx))
    err(it).rnn = y(end-199:end,idx) - y_hat_rnn(end-199:end,idx);
    err(it).spl = y(end-199:end,idx) - y_hat_spl(end-199:end,idx);
end
legend('y','y rnn', 'y spl')

% mean(reshape([err(:).rnn],[],1))
% std(reshape([err(:).rnn],[],1))
% 
% mean(reshape([err(:).spl],[],1))
% std(reshape([err(:).spl],[],1))