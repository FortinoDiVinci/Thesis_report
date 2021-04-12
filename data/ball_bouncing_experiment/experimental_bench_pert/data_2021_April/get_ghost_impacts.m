function cycles_new = get_ghost_impacts(cycles, ghost_impact_t)

    cycles_new = cycles;
    
    for trial_id = 1:length(cycles)
        for n_cycle = 1:length(cycles{trial_id})
            %
            if any(isnan(ghost_impact_t{trial_id}))
                continue
            end
            cycles_new{trial_id}(n_cycle).is_ghost_impact = any(...
                (cycles{trial_id}(n_cycle).t(1) < ghost_impact_t{trial_id}) & ...
                (cycles{trial_id}(n_cycle).t(end) > ghost_impact_t{trial_id}));
        end
    end

end

