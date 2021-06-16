function cycles_n = disp_temporal_norm_multp_cond(cycles, names)

if length(unique(names)) < 7
    colors = lines(length(unique(names)));
else
   colors = hsv(length(unique(names)));
end
%colors = parula(length(unique(names))); % hsv
dot_colors = parula(length(unique([cycles{1}.type_dist]))-1);  

%% mean and std
tocm = 100;
cycles_n = cycles;
names_regr = names;
regrouped = [];
for ii = 1:length(cycles)
    if any(ii == regrouped)
        names_regr(ii) = "";
        continue
    end
    tmp = names;
    tmp(ii) = [""]; % mask the iith name to avoid seing itself as duplicate
    string_compare = strcmp(names(ii), tmp);
    idx = find(string_compare);
    %cycles_n{ii} = cycles{ii};
    if any(string_compare)
        for i_idx = idx'
            regrouped = [regrouped, i_idx];
            cycles_n{ii} = horzcat(cycles_n{ii}, cycles{i_idx});
            cycles_n{[i_idx]} = [];
        end
        %cycles_n([idx]) = cell(1,length(idx));
    end
    idx_is_dist = ~logical(vertcat(cycles_n{ii}(:).is_dist));
    avg_cycle_pos(ii,:) = mean(vertcat(cycles_n{ii}(idx_is_dist).ui_interp));
    std_cycle_pos(ii,:) = std(vertcat(cycles_n{ii}(idx_is_dist).ui_interp));
    avg_cycle_for(ii,:) = mean(vertcat(cycles_n{ii}(idx_is_dist).yi_interp));
    std_cycle_for(ii,:) = std(vertcat(cycles_n{ii}(idx_is_dist).yi_interp));
    % normalisation magnitude
    avg_cycle_pos_mag(ii) = mean(vertcat(cycles_n{ii}(idx_is_dist).ui_max)).*tocm;
    std_cycle_pos_mag(ii) = std(vertcat(cycles_n{ii}(idx_is_dist).ui_max)).*tocm;
    avg_cycle_for_mag(ii) = mean(vertcat(cycles_n{ii}(idx_is_dist).yi_max));
    std_cycle_for_mag(ii) = std(vertcat(cycles_n{ii}(idx_is_dist).yi_max));
end
% deleting empty lines
avg_cycle_pos = avg_cycle_pos(~(names_regr==""),:);
std_cycle_pos = std_cycle_pos(~(names_regr==""),:);
avg_cycle_for = avg_cycle_for(~(names_regr==""),:);
std_cycle_for = std_cycle_for(~(names_regr==""),:);
%
avg_cycle_pos_mag = avg_cycle_pos_mag(~(names_regr==""));
std_cycle_pos_mag = std_cycle_pos_mag(~(names_regr==""));
avg_cycle_for_mag = avg_cycle_for_mag(~(names_regr==""));
std_cycle_for_mag = std_cycle_for_mag(~(names_regr==""));
%
names_regr = names_regr(~(names_regr==""));
cycles_n =  cycles_n(~cellfun('isempty',cycles_n));

figure('DefaultAxesFontSize',16)
% subplot(1,2,1)
tiledlayout(1,2, 'TileSpacing', 'compact', 'Padding', 'none')
nexttile
hold on
for ii = 1:length(cycles_n)
%     p(ii) = plot(vertcat(cycles_n{1}(1).t_com)', avg_cycle_pos(ii,:).*avg_cycle_pos_mag(ii),...
%     '--', 'Color', colors(ii,:), 'Linewidth', 1.5);
    plotStdSurface((avg_cycle_pos(ii,:).*avg_cycle_pos_mag(ii))', ...
        (std_cycle_pos(ii,:).*avg_cycle_pos_mag(ii))', vertcat(cycles_n{1}(1).t_com), ...
        colors(ii,:), 3);
%     plotStdSurface((avg_cycle_pos(ii,:).*avg_cycle_pos_mag(ii))', ...
%         (std_cycle_pos(ii,:).*avg_cycle_pos_mag(ii))', vertcat(cycles{1}(1).t_com), ...
%         colors(ii,:), 1)
end
%legend([p(1), p(2),p(3),p(4)],{'test', 'test', 'test', 'test'})
% perturbations
for ii = 1:max(cellfun(@(x) max([x.type_dist]), cycles_n))
    r(ii) = mean([cycles_n{1}([cycles_n{1}.type_dist] == ii).ratio_dist]);
end
[~,idxs] = sort(r);
[~,idxs_s] = sort(idxs);
% plots pert timing on position
for exp_nb = 1:length(cycles_n)
for ii = 1:length(r)
    cyc_pert = find([cycles_n{exp_nb}.type_dist] == ii);
    % perturbation index for each perturbed cycle
    idx_pert = floor([cycles_n{exp_nb}(cyc_pert).ratio_dist].*...
        length(cycles_n{exp_nb}(1).t_com));
        if any(idx_pert == 0)
            idx_pert(idx_pert == 0) = 1; % avoid this rare scenario of 0 idx
        end
        for j = 1:length(idx_pert) 
            scatter(cycles_n{exp_nb}(cyc_pert(j)).t_com(idx_pert(j)),...
                cycles_n{exp_nb}(cyc_pert(j)).ui_interp(idx_pert(j)).*avg_cycle_pos_mag(exp_nb), ...
                'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', ...
                dot_colors(end-ii+1,:)); 
        end
end
end
for ii = 1:length(cycles_n)
    p(ii) = plot(vertcat(cycles_n{1}(1).t_com)', avg_cycle_pos(ii,:).*avg_cycle_pos_mag(ii),...
    '--', 'Color', colors(ii,:), 'Linewidth', 1.5);
end
legend(p,cellstr(names_regr))
%
% for i = 1:length(cycles)
%     for k = 1:3
%         cyc_pert = find([cycles{i}.type_dist] == k);
%         %idx_pert = [cycles{i}(cyc_pert).ind];
%         idx_pert = floor([cycles{i}(cyc_pert).ratio_dist].*length(cycles{i}(1).t_com));
%         if any(idx_pert == 0)
%             idx_pert(idx_pert == 0) = 1; % avoid this rare scenario of 0 idx
%         end
%         for j = 1:length(idx_pert) 
%         scatter(cycles{i}(cyc_pert(j)).t_com(idx_pert(j)),...
%             cycles{i}(cyc_pert(j)).ui_interp(idx_pert(j)), ...
%             'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', colors(k,:)); 
%         end
%     end
% end
xlabel('Normalized time')
ylabel('Position (cm)')
%subplot(1,2,2)
nexttile
hold on
for ii = 1:length(cycles_n)
%     p(ii) = plot(vertcat(cycles_n{1}(1).t_com)', avg_cycle_for(ii,:).*avg_cycle_for_mag(ii),...
%     '--', 'Color', colors(ii,:), 'Linewidth', 1.5);
    plotStdSurface((avg_cycle_for(ii,:).*avg_cycle_for_mag(ii))', ...
        (std_cycle_for(ii,:).*avg_cycle_for_mag(ii))', vertcat(cycles_n{1}(1).t_com), ...
        colors(ii,:), 3);
%     plotStdSurface((avg_cycle_for(ii,:).*avg_cycle_for_mag(ii))', ...
%         (std_cycle_for(ii,:).*avg_cycle_for_mag(ii))', vertcat(cycles{1}(1).t_com), ...
%         colors(ii,:), 1)
end
for exp_nb = 1:length(cycles_n)
for ii = 1:length(r)
    cyc_pert = find([cycles_n{exp_nb}.type_dist] == ii);
    % perturbation index for each perturbed cycle
    idx_pert = floor([cycles_n{exp_nb}(cyc_pert).ratio_dist].*...
        length(cycles_n{exp_nb}(1).t_com));
        if any(idx_pert == 0)
            idx_pert(idx_pert == 0) = 1; % avoid this rare scenario of 0 idx
        end
        for j = 1:length(idx_pert) 
            scatter(cycles_n{exp_nb}(cyc_pert(j)).t_com(idx_pert(j)),...
                cycles_n{exp_nb}(cyc_pert(j)).yi_interp(idx_pert(j)).*avg_cycle_for_mag(exp_nb), ...
                'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', ...
                dot_colors(end-ii+1,:)); 
        end
end
end
for ii = 1:length(cycles_n)
    p(ii) = plot(vertcat(cycles_n{1}(1).t_com)', avg_cycle_for(ii,:).*avg_cycle_for_mag(ii),...
    '--', 'Color', colors(ii,:), 'Linewidth', 1.5);
end
legend(p,cellstr(names_regr))
%
% for i = 1:length(cycles)
%     for k = 1:3
%         cyc_pert = find([cycles{i}.type_dist] == k);
%         %idx_pert = [cycles{i}(cyc_pert).ind];
%         idx_pert = floor([cycles{i}(cyc_pert).ratio_dist].*length(cycles{i}(1).t_com));
%         if any(idx_pert == 0)
%             idx_pert(idx_pert == 0) = 1; % avoid this rare scenario of 0 idx
%         end
%         for j = 1:length(idx_pert) 
%         scatter(cycles{i}(cyc_pert(j)).t_com(idx_pert(j)),...
%             cycles{i}(cyc_pert(j)).yi_interp(idx_pert(j)), ...
%             'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', colors(k,:)); 
%         end
%     end
% end
xlabel('Normalized time')
ylabel('Force (N)')
