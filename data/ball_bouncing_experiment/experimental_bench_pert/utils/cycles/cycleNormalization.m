function [norm_cycles, min_idx, max_amp_pos, max_amp_vel, max_amp_for] = cycleNormalization(cycles, doTimeNorm)
%cycle_data_norm
%   Input : CYCLE_DATA array
%           BOOL 
%   Outputs : 
%       - CYCLE_DATA array
%       - min index used for the time normalization
%       - max position magnitude
%       - max velocity magnitude
%       - max force magnitude
    
    if nargin < 2
        % default behaviour does time normalization
        doTimeNorm = 1;
    end

    min_idx = inf;
    max_amp_pos = 0;
    max_amp_vel = 0;
    max_amp_for = 0;
    for cyc = cycles
        min_idx_cyc = length(cyc.idx);
        max_amp_pos_cyc = max(cyc.position) - min(cyc.position);
        max_amp_vel_cyc = max(cyc.velocity) - min(cyc.velocity);
        max_amp_for_cyc = max(cyc.force) - min(cyc.force);
        if min_idx_cyc < min_idx
            min_idx = min_idx_cyc;
        end
        if max_amp_pos_cyc > max_amp_pos
            max_amp_pos = max_amp_pos_cyc;
        end
        if max_amp_vel_cyc > max_amp_vel
            max_amp_vel = max_amp_vel_cyc;
        end
        if max_amp_for_cyc > max_amp_for
            max_amp_for = max_amp_for_cyc;
        end
    end
    
    %security if not all data were provided in the cycle
    if max_amp_pos == 0
        NULL_POS = 1;
    else
        NULL_POS = 0;
    end
    
    if max_amp_vel == 0
        NULL_VEL = 1;
    else
        NULL_VEL = 0;
    end
    
    if max_amp_for == 0
        NULL_FOR = 1;
    else
        NULL_FOR = 0;
    end
    
    for ii = length(cycles):-1:1
        norm_cycles(ii) = cycles(ii).copy(); % other wise modifying one changes the other
        if logical(doTimeNorm)
            norm_cycles(ii).time = linspace(norm_cycles(ii).time(1), norm_cycles(ii).time(end), min_idx);
        end
        if ~NULL_POS
            norm_cycles(ii).position = norm_cycles(ii).position./max_amp_pos;
            if logical(doTimeNorm)
                norm_cycles(ii).position = interp1(cycles(ii).time, norm_cycles(ii).position, norm_cycles(ii).time);
            end
        end
        if ~NULL_VEL
            norm_cycles(ii).velocity = norm_cycles(ii).velocity./max_amp_vel; 
            if logical(doTimeNorm)
                norm_cycles(ii).velocity = interp1(cycles(ii).time, norm_cycles(ii).velocity, norm_cycles(ii).time);
            end
        end
        if ~NULL_FOR
            norm_cycles(ii).force = norm_cycles(ii).force./max_amp_for;   
            if logical(doTimeNorm)
                norm_cycles(ii).force = interp1(cycles(ii).time, norm_cycles(ii).force, norm_cycles(ii).time);
            end
        end
        if logical(doTimeNorm)
            norm_cycles(ii).time = linspace(0, 1, min_idx);
        end
    end
    
end

