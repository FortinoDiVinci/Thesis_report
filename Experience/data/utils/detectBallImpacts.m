function ind_i = detectBallImpacts(t, zb, disp)

dzb = Iu_diffcent(zb, t); % centered differentiation
ind_i1 = crossing(dzb); % indexes where zb derivative passes through 0

ind_i1(ind_i1 + 5 > length(zb)) = []; % to avoid overflow (last bounce are not used anw
ind_i = ind_i1(dzb([ind_i1] + 5) > 0); % idx with following positive velocity

if nargin > 2
    if disp
        figure
        hold on
        plot(t, zb, 'Linewidth', 2)
        plot(t(ind_i), zb(ind_i), '*', 'Markersize',15)
    end
end

end
