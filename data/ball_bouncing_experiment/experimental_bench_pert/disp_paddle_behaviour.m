
i = 1;

val_dist = dist{i}(1:2:end);
tim_dist = t_dist{i}(1:2:end);

idx = NaN(length(tim_dist), 1);

for dist_nb = 1:length(tim_dist)   
    idx(dist_nb) = find(t{i} >= tim_dist(dist_nb), 1, 'first');   
end

figure
plot(t{i}, (z{i}-0.325)*6, 'b', 'Linewidth',4)
hold on
plot(t{i}, z_b{i}, ':r', 'Linewidth',4)
plot(tim_dist, (z{i}(idx)-0.325)*6, 'k.','MarkerSize',25)
ax = gca;
ax.XAxis.FontSize = 35;
ax.YAxis.FontSize = 35;

figure
plot(t{i}, fz{i}, 'b', 'Linewidth',4)
hold on
plot(tim_dist, fz{i}(idx), 'k.','MarkerSize',25)
ax = gca;
ax.XAxis.FontSize = 35;
ax.YAxis.FontSize = 35;