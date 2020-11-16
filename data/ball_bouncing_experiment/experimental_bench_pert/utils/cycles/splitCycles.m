function cycles_list = splitCycles(positions, forces, times, cut_idx, pert_idx, pert_val)
% Creates a cycle list from position and force data, according to the
% indexes called cut_idx
    
    % provided either one of position and force is possible
    if all([isempty(positions), isempty(forces)])
        error("No data provided")
    elseif isempty(positions)
        warning("No position provided")
        POS_NULL=1;
        FOR_NULL=0;
    elseif isempty(forces)
        warning("No position provided")
        POS_NULL=0;
        FOR_NULL=1;
    else
        POS_NULL=0;
        FOR_NULL=0;
    end
    p = [];
    f = [];
    for i = length(cut_idx):-1:2 % for preallocation we start at the end
        idx = (cut_idx(i-1):cut_idx(i));
        if ~POS_NULL
            p = positions(idx);
        end
        if ~FOR_NULL
            f   = forces(idx);
        end
        t   = times(idx);
        cycles_list(i-1) = CYCLE_DATA(t, p, [], f, idx);
    end
    
    % specify the cycles that are perturbed
    if nargin >= 5 
        for cycle = cycles_list
            bool_pert_cyc = logical(cycle.time(1) < times(pert_idx)) & logical(cycle.time(end) > times(pert_idx));
            if(any(bool_pert_cyc))
                cycle.setPerturbation(pert_val(bool_pert_cyc), times(pert_idx(bool_pert_cyc)));
            end
        end
    end
    
end

