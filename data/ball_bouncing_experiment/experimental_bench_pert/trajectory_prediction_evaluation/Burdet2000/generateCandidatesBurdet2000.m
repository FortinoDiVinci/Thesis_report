function cand = generateCandidatesBurdet2000(forces_list, magn_mult_list, time_dist_list, time_shift_list)

    % mean computation
    % /!\ In the original algorithm the mean is computed independantly for 
    % the positive and negative acceleration part, but here, multiple cases
    % happens and the average is therefore not computed independantly
    avg_force = mean(forces_list,1);
    std_force = max(std(forces_list));
    
    m_nb = length(magn_mult_list);
    td_nb = length(time_dist_list);
    ts_nb = length(time_shift_list);
    size_traj = size(forces_list,2);
    
    cand = NaN(m_nb*td_nb*ts_nb, size_traj);
    
    % candidates population
    for ii = 1:td_nb % time distortions
        cand_dist = generateTimeDistortion(avg_force, (1:size_traj), time_dist_list(ii));
        for jj = 1:m_nb % magnitude amplifications
            cand_magn = cand_dist.*magn_mult_list(jj);
            for kk = 1:ts_nb % time shifts
                if time_shift_list(kk) > 0
                    cand_shift = cat(2,cand_magn(1+time_shift_list(kk):end), ...
                        NaN(1,time_shift_list(kk))); % complete the missing data with NaN
                elseif time_shift_list(kk) < 0
                    cand_shift = cat(2,NaN(1,-time_shift_list(kk)), ...
                        cand_magn(1:end+time_shift_list(kk)));
                else
                    cand_shift = cand_magn(1+time_shift_list(kk):end);
                end
                cand(ts_nb*(m_nb*(ii-1)+jj-1)+kk,:) = cand_shift; % result
            end % time shifts
        end % magnitude amplifications
    end % time distortions
    
end

