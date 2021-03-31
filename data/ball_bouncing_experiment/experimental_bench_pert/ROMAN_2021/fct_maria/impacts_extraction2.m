function ind_i = impacts_extraction2(t, zb, thresh)

dzb = Iu_diffcent(zb, t); % centered differentiation

ind_i1 = crossing(dzb); % indices où la dérivée de zb passe par 0
ind_i2 = find(zb(ind_i1) < thresh); % indices parmi ind_i1 où zb < 0.5
ind_i = ind_i1(ind_i2); % indices d'impacts parmi le temps total

figure
hold on
plot(t, zb)
plot(t(ind_i), zb(ind_i),'*')

end