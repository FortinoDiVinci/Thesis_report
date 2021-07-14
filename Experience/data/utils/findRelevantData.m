function [i_red,idx_i_red] = findRelevantData(t, z, i_impact, first_impact)


t_red1 = t(i_impact(first_impact)); 
% the last seconds of the experiment might be irrelevant
% we try to detect non cyclic behaviour 
period = diff(t(i_impact));
for i = length(i_impact) - 1:-1:1
    cyc = z(i_impact(i):i_impact(i+1));
    mag(i) = abs(max(cyc) - min(cyc));
end
avg_period = mean(period);
avg_mag = mean(mag);
for i = length(i_impact) - 1:-1:1
    % if the period or the magnitude is too low at the end, it is probably
    % a non cyclic behaviour or a pathological behaviour
    if period(i) < 0.5*avg_period || mag(i) < 0.5*avg_mag
        continue
    else
        last_impact = i;
        break
    end
end

% as a margin of error, 3 more cycles are deleted
t_red2 = t(i_impact(last_impact-3)); 

idx_i_red = i_impact(first_impact:last_impact-3);
i_red = find((t >= t_red1) & (t <= t_red2)); 