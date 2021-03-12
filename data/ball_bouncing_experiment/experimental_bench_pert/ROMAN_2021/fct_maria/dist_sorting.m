function [C_ratio, count_ratio, idx_ratio, cycles_class] = dist_sorting(cycles_norm)

ratio_dist = [];
for k = 1:length(cycles_norm)
    %ratio_dist = [ratio_dist; cellfun(@(S)(S.ratio_dist), cycles_norm{k})'];
    ratio_dist = [ratio_dist; [cycles_norm{k}.ratio_dist]'];
end


ind_dist_p = find(ratio_dist > 0);
ratio_dist_p = ratio_dist(ind_dist_p);

[idx_ratio_p, C_ratio] = kmeans(ratio_dist_p, 3); % clustering des perturbations selon l'instant dans le cycle 
idx_ratio = zeros(length(ratio_dist),1);
idx_ratio(ind_dist_p) = idx_ratio_p;

% C_ratio % centres des clusters
count_ratio = [sum(idx_ratio==1) sum(idx_ratio==2) sum(idx_ratio==3)]; % nb d'expé par cluster

pts = 1:length(ratio_dist);
figure
plot(pts, ratio_dist, 'k*'); hold on
plot(pts(idx_ratio==1), ratio_dist(idx_ratio==1), 'ro')
plot(pts(idx_ratio==2), ratio_dist(idx_ratio==2), 'bo')
plot(pts(idx_ratio==3), ratio_dist(idx_ratio==3), 'go')

cycles_class = cycles_norm;
count = 1;
for trial_id = 1:length(cycles_class)
    for n_cycle = 1:length(cycles_class{trial_id})

            cycles_class{trial_id}(n_cycle).type_dist = idx_ratio(count);
            count = count+1;
            
    end
end


