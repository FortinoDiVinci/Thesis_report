function cycles_norm = cycleNormalisation(cycles, dt)

nb_exp = length(cycles);

tmp = cellfun(@(x) vertcat(x.npts), cycles, 'UniformOutput', false);
Lmean = mean(rmoutliers(vertcat(tmp{:})));

for trial_id = 1:nb_exp
    
    cycle_id = cycles{trial_id};

    t_com = [0:Lmean-1]*dt;
    t_com = t_com/max(t_com);
    
    for i = 1:length(cycle_id)
        
%       cycle = i
        cycle_id(i).ui_norm = cycle_id(i).ui/cycle_id(i).ui_max;
        cycle_id(i).ui_norm = cycle_id(i).ui_norm - cycle_id(i).ui_norm(1);
        cycle_id(i).dui_norm = cycle_id(i).dui/cycle_id(i).dui_max;
        cycle_id(i).dui_norm = cycle_id(i).dui_norm - cycle_id(i).dui_norm(1);
        
        cycle_id(i).yi_norm = cycle_id(i).yi/cycle_id(i).yi_max;
        cycle_id(i).yi_norm = cycle_id(i).yi_norm - cycle_id(i).yi_norm(1);
        cycle_id(i).dyi_norm = cycle_id(i).dyi/cycle_id(i).dyi_max;
        cycle_id(i).dyi_norm = cycle_id(i).dyi_norm - cycle_id(i).dyi_norm(1);
        
        cycle_id(i).t_norm = cycle_id(i).t_red/max(cycle_id(i).t_red);
        
        cycle_id(i).ui_interp = interp1(cycle_id(i).t_norm, cycle_id(i).ui_norm, t_com);
        cycle_id(i).dui_interp = interp1(cycle_id(i).t_norm, cycle_id(i).dui_norm, t_com);
        cycle_id(i).yi_interp = interp1(cycle_id(i).t_norm, cycle_id(i).yi_norm, t_com);
        cycle_id(i).dyi_interp = interp1(cycle_id(i).t_norm, cycle_id(i).dyi_norm, t_com);
        
        cycle_id(i).t_com = t_com;
        
    end %-- end for cycles i
    
    cycles_norm{trial_id} = cycle_id;
    
end %-- end for exp

