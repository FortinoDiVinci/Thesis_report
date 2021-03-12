function [i_red,ind_i_red] = boundaries_suppr(t, i_imp, first_impact, fct_time_ratio)

time = fct_time_ratio*(t(end) - t(i_imp(first_impact))); % s
t_red1 = t(i_imp(first_impact)); 
t_red2 = t_red1(1)+ time; 
i_red = find((t >= t_red1) & (t <= t_red2)); % tous indices de t sans bords
last_impact = crossing(i_imp - find(t > t_red2,1)); % dernier impact sur la durée utile d'essai
ind_i_red = i_imp(first_impact:last_impact); % indices d'impacts dans t, sans les bords

