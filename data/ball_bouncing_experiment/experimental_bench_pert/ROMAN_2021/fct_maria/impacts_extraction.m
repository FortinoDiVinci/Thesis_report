function ind_i = impacts_extraction(t, z_b, z_p, thresh)

diff_zbp = z_b - z_p; % écart pos balle et pos raquette
ddiff_zbp = Iu_diffcent(diff_zbp, t); % centered differentiation


ind_i1 = crossing(ddiff_zbp); % indices où la dérivée de l'écart passe par 0
ind_i2 = find(diff_zbp(ind_i1) < thresh); % indices parmi ind_i1 où l'écart est négatif
ind_i = ind_i1(ind_i2); % indices d'impacts parmi le temps total

figure
hold on
plot(t, diff_zbp)
plot(t(ind_i), diff_zbp(ind_i),'*')

end