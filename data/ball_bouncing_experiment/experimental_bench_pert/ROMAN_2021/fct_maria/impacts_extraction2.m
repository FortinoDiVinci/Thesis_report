function ind_i = impacts_extraction2(t, zb, thresh)

dzb = Iu_diffcent(zb, t); % centered differentiation

ind_i1 = crossing(dzb); % indexes where zb derivative passes through 0
% This method is not robust enough
if nargin > 2
    ind_i2 = find(zb(ind_i1) < thresh); % indexes ind_i1 with z_b < 0.5
else
    ind_i1(ind_i1 + 5 > length(zb)) = []; % to avoid overflow (last bounce are not used anw
    ind_i2 = find(dzb([ind_i1] + 5) > 0); % idx with following positive velocity
end
ind_i = ind_i1(ind_i2); % indices d'impacts parmi le temps total

figure
hold on
plot(t, zb, 'Linewidth', 2)
plot(t(ind_i), zb(ind_i), '*', 'Markersize',15)

end