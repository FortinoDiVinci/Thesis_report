function [cycles_mean, cycles_std] = getDistribution(cycles, varargin)
%TODO
%   Detailed explanation goes here

    cycle_mean_pos = NaN(length(cycles(1).time), 1);
    cycle_std_pos = NaN(length(cycles(1).time), 1);
    cycle_mean_vel = NaN(length(cycles(1).time), 1);
    cycle_std_vel = NaN(length(cycles(1).time), 1);
    
    for idx = length(cycles(1).time):-1:1 % this way the object vector is preallocated     
        positions_at_idx = NaN(length(cycles),1);
        velocities_at_idx = NaN(length(cycles),1);
        for c_n = 1:length(cycles)
            positions_at_idx(c_n) = cycles(c_n).position(idx);
            velocities_at_idx(c_n) = cycles(c_n).velocity(idx);
        end
        position_statistics(idx) = STATISTIC_DATA(positions_at_idx, 'positions', 'cm');
        position_statistics(idx).compute_new_statistics(); 
        velocity_statistics(idx) = STATISTIC_DATA(velocities_at_idx, 'velocities', 'm/s');
        velocity_statistics(idx).compute_new_statistics(); 
        
        cycle_mean_pos(idx) = position_statistics(idx).mean(1);
        cycle_std_pos(idx) = position_statistics(idx).std_dev(1);
        cycle_mean_vel(idx) = velocity_statistics(idx).mean(1);
        cycle_std_vel(idx) = velocity_statistics(idx).std_dev(1);
        
    end
    
    cycles_mean = CYCLE_DATA(cycles(1).time, cycle_mean_pos, cycle_mean_vel); 
    cycles_std = CYCLE_DATA(cycles(1).time, cycle_std_pos, cycle_std_vel);
    
    DO_STAT_PER_PERT_DIRECTION = 0;
    if ~isempty(varargin)
        for ii = 1:2:length(varargin)
            switch(varargin{ii})
                case 'PertDir'
                    if varargin{ii+1} == 'y' || varargin{ii+1} == 1
                        DO_STAT_PER_PERT_DIRECTION = 1;
                    end
                otherwise
                    %
                    warning('unknown argument')
            end
        end
    end
    
    if DO_STAT_PER_PERT_DIRECTION
    % Statistics per perturbations directions
        pert_dir = [];
        data_pos = {};
        data_vel = {};
        pert_dir = [pert_dir, cycles.pert_dir];       
        data_pos = {cycles.position};
        data_vel = {cycles.velocity};
        
        data_pos_no_pert = data_pos(logical(pert_dir == 0));
        data_pos_pos_pert = data_pos(logical(pert_dir == 1));
        data_pos_neg_pert = data_pos(logical(pert_dir == -1));
        data_vel_no_pert = data_vel(logical(pert_dir == 0));
        data_vel_pos_pert = data_vel(logical(pert_dir == 1));
        data_vel_neg_pert = data_vel(logical(pert_dir == -1));
    end
    
end

