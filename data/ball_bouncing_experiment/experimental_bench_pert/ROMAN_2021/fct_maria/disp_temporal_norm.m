function disp_temporal_norm(cycles)

colors = [0.8500, 0.3250, 0.0980;
          0.9290, 0.6940, 0.1250;
          0.4940, 0.1840, 0.5560];

      %modif avec ratio dist
      
figure('DefaultAxesFontSize',13)
subplot(1,2,1)
hold on
for i = 1:length(cycles)
plot(vertcat(cycles{i}(:).t_com)', vertcat(cycles{i}(:).ui_interp)', ...
    'Color', [0, 0.4470, 0.7410, 0.2])
end
for i = 1:length(cycles)
    for k = 1:3
        cyc_pert = find([cycles{i}.type_dist] == k);
        %idx_pert = [cycles{i}(cyc_pert).ind];
        idx_pert = floor([cycles{i}(cyc_pert).ratio_dist].*length(cycles{i}(1).t_com));
        for j = 1:length(idx_pert) 
        scatter(cycles{i}(cyc_pert(j)).t_com(idx_pert(j)),...
            cycles{i}(cyc_pert(j)).ui_interp(idx_pert(j)), ...
            'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', colors(k,:)); 
        end
    end
end
xlabel('Normalized time')
ylabel('Normalized Position')
subplot(1,2,2)
hold on
for i = 1:length(cycles)
plot(vertcat(cycles{i}(:).t_com)', vertcat(cycles{i}(:).yi_interp)', ...
    'Color', [0, 0.4470, 0.7410, 0.2])
end
for i = 1:length(cycles)
    for k = 1:3
        cyc_pert = find([cycles{i}.type_dist] == k);
        %idx_pert = [cycles{i}(cyc_pert).ind];
        idx_pert = floor([cycles{i}(cyc_pert).ratio_dist].*length(cycles{i}(1).t_com));
        for j = 1:length(idx_pert) 
        scatter(cycles{i}(cyc_pert(j)).t_com(idx_pert(j)),...
            cycles{i}(cyc_pert(j)).yi_interp(idx_pert(j)), ...
            'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', colors(k,:)); 
        end
    end
end
xlabel('Normalized time')
ylabel('Normalized Force')

%% mean and std
all_cycles_pos = [];
all_cycles_for = [];
for ii = 1:length(cycles)
    all_cycles_pos = [all_cycles_pos, vertcat(cycles{ii}(:).ui_interp)'];
    all_cycles_for = [all_cycles_for, vertcat(cycles{ii}(:).yi_interp)'];
end
avg_cycle_pos = mean(all_cycles_pos,2);
std_cycle_pos = std(all_cycles_pos,1,2);
avg_cycle_for = mean(all_cycles_for,2);
std_cycle_for = std(all_cycles_for,1,2);

figure('DefaultAxesFontSize',13)
subplot(1,2,1)
hold on
plot(vertcat(cycles{1}(1).t_com)', avg_cycle_pos, '--', ...
    'Color', [0, 0.4470, 0.7410], 'Linewidth', 1.5);
plotStdSurface(avg_cycle_pos, std_cycle_pos, vertcat(cycles{1}(1).t_com), [0, 0.4470, 0.7410], 3)
plotStdSurface(avg_cycle_pos, std_cycle_pos, vertcat(cycles{1}(1).t_com), [0, 0.4470, 0.7410], 1)
%
for i = 1:length(cycles)
    for k = 1:3
        cyc_pert = find([cycles{i}.type_dist] == k);
        %idx_pert = [cycles{i}(cyc_pert).ind];
        idx_pert = floor([cycles{i}(cyc_pert).ratio_dist].*length(cycles{i}(1).t_com));
        for j = 1:length(idx_pert) 
        scatter(cycles{i}(cyc_pert(j)).t_com(idx_pert(j)),...
            cycles{i}(cyc_pert(j)).ui_interp(idx_pert(j)), ...
            'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', colors(k,:)); 
        end
    end
end
xlabel('Normalized time')
ylabel('Normalized Position')
subplot(1,2,2)
hold on
plot(vertcat(cycles{1}(1).t_com)', avg_cycle_for, '--', ...
    'Color', [0, 0.4470, 0.7410], 'Linewidth', 1.5);
plotStdSurface(avg_cycle_for, std_cycle_for, vertcat(cycles{1}(1).t_com), [0, 0.4470, 0.7410], 3)
plotStdSurface(avg_cycle_for, std_cycle_for, vertcat(cycles{1}(1).t_com), [0, 0.4470, 0.7410], 1)
%
for i = 1:length(cycles)
    for k = 1:3
        cyc_pert = find([cycles{i}.type_dist] == k);
        %idx_pert = [cycles{i}(cyc_pert).ind];
        idx_pert = floor([cycles{i}(cyc_pert).ratio_dist].*length(cycles{i}(1).t_com));
        for j = 1:length(idx_pert) 
        scatter(cycles{i}(cyc_pert(j)).t_com(idx_pert(j)),...
            cycles{i}(cyc_pert(j)).yi_interp(idx_pert(j)), ...
            'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', colors(k,:)); 
        end
    end
end
xlabel('Normalized time')
ylabel('Normalized Force')
