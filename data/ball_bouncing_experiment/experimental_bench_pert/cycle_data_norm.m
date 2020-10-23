function [norm_cycles, max_idx, max_amp_pos, max_amp_vel] = cycle_data_norm(cycles)
%cycle_data_norm
%   Input : CYCLE_DATA array
%   Outputs : 
%       - CYCLE_DATA array
%       - max index used for the time normalization
%       - max position magnitude
%       - max velocity magnitude
    
    min_idx = inf;
    max_amp_pos = 0;
    max_amp_vel = 0;
    for cyc = cycles
        min_idx_cyc = length(cyc.idx);
        max_amp_pos_cyc = max(cyc.position) - min(cyc.position);
        max_amp_vel_cyc = max(cyc.velocity) - min(cyc.velocity);
        if min_idx_cyc < min_idx
            min_idx = min_idx_cyc;
        end
        if max_amp_pos_cyc > max_amp_pos
            max_amp_pos = max_amp_pos_cyc;
        end
        if max_amp_vel_cyc > max_amp_vel
            max_amp_vel = max_amp_vel_cyc;
        end
    end
    
    for ii = length(cycles):-1:1
        norm_cycles(ii) = cycles(ii).copy(); % other wise modifying one changes the other
        norm_cycles(ii).position = norm_cycles(ii).position./max_amp_pos;
        norm_cycles(ii).velocity = norm_cycles(ii).velocity./max_amp_vel; 
        norm_cycles(ii).time = linspace(norm_cycles(ii).time(1), norm_cycles(ii).time(end), min_idx);
        norm_cycles(ii).position = interp1(cycles(ii).time, norm_cycles(ii).position, norm_cycles(ii).time);
        norm_cycles(ii).velocity = interp1(cycles(ii).time, norm_cycles(ii).velocity, norm_cycles(ii).time);
        norm_cycles(ii).time = linspace(0, 1, min_idx);
    end
    
end

