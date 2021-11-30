function [C_ratio, count_ratio, idx_ratio, cycles_class] = sortPerturbations(cycles_norm, nb_clusters, display)

ratio_dist = [];
for k = 1:length(cycles_norm)
    ratio_dist = [ratio_dist; [cycles_norm{k}.ratio_dist]'];
end

ind_dist_p = find(ratio_dist > 0);
ratio_dist_p = ratio_dist(ind_dist_p);

[idx_ratio_p, C_ratio] = kmeans(ratio_dist_p, nb_clusters); % clustering
idx_ratio = zeros(length(ratio_dist),1);
idx_ratio(ind_dist_p) = idx_ratio_p;

% C_ratio % clusters centers
for ii = 1:nb_clusters
    count_ratio(ii) = sum(idx_ratio==ii); % nb exp per cluster
end

pts = 1:length(ratio_dist);
if display
    figure
    plot(pts, ratio_dist, 'k*'); hold on
    for ii = 1:nb_clusters
        plot(pts(idx_ratio==ii), ratio_dist(idx_ratio==ii), 'o')
    end
    ylim([0,1])
end

cycles_class = cycles_norm;
count = 1;
for trial_id = 1:length(cycles_class)
    for n_cycle = 1:length(cycles_class{trial_id})
        cycles_class{trial_id}(n_cycle).type_dist = idx_ratio(count);
%         if idx_ratio(count) ~= 0 
%             cycles_class{trial_id}(n_cycle).type_dist = C_ratio(idx_ratio(count));
%         else
%             cycles_class{trial_id}(n_cycle).type_dist = 0;
%         end
        count = count+1;            
    end
end
