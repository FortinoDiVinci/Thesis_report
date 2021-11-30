function cycle_iout = cycleExtraction(ind_i_red, dt, dfz, dz, z_b, z_p, exp_parameters, display)

% -- to acces a field of the struct through cycles: 
% -- C = max(cellfun(@(S)(S.npts),cycle_i)) 

cycle_i = {};

d_dz = Iu_diffcent(dz.complete_traject, dz.time);
d_dfz = Iu_diffcent(dfz.complete_traject, dfz.time);
dz_b = Iu_diffcent(z_b, dz.time);

for i = 1:length(ind_i_red) - 1
    
    cycle_i(i).n = i;
    cycle_i(i).user = string(exp_parameters.user);
    try
        cycle_i(i).exp = exp_parameters.experience;
    catch
        warning("Experience type was not specified");
    end
    cycle_i(i).target_height = exp_parameters.target_height;
    
    %--- time and duration processing
    ind_pts_i = [ind_i_red(i):ind_i_red(i+1) - 1]'; % absolute index in cycle
    cycle_i(i).npts = length(ind_pts_i); % nb of pts in cycle
    cycle_i(i).t_red = [0:cycle_i(i).npts-1]'*dt; % cycle time [0:tend]
    cycle_i(i).duration = max(cycle_i(i).t_red); % cycle duration
    
    %--- indexes and perturbations processing
    [ind_dist_i,~,IB] = intersect(ind_pts_i, dfz.pert_ind); % abs idx start and end of pert
    cycle_i(i).ind = ind_dist_i - ind_i_red(i); % relative idx start and end of pert
    cycle_i(i).glob_ind = ind_dist_i; % global idx
    cycle_i(i).is_dist = ~isempty(ind_dist_i); % perturbed cycle ?
    if i > 1
        cycle_i(i).is_post_dist = (cycle_i(i-1).is_dist) == 1; % previous cycle perturbed
    else
        cycle_i(i).is_post_dist = 0; % i = 1
    end
    
    cycle_i(i).impact_vel = d_dz(ind_pts_i(1)); % velocity at impact
    cycle_i(i).impact_pos = dz.complete_traject(ind_pts_i(1));
    cycle_i(i).actual_target_height = cycle_i(i).target_height - cycle_i(i).impact_pos;
    cycle_i(i).target_error = z_b(ind_pts_i(out2(@() max(z_b(ind_pts_i))))) - cycle_i(i).target_height;
    
    if cycle_i(i).is_dist
        cycle_i(i).dist_val = dfz.pert_val(IB(1));
        cycle_i(i).ratio_dist = cycle_i(i).ind(1)./cycle_i(i).npts; % normalised perturbation ratio
    else
        cycle_i(i).dist_val = 0;
        cycle_i(i).ratio_dist = -1;
    end
    
    if cycle_i(i).dist_val > 0
        cycle_i(i).sign_dist = 1;
    elseif cycle_i(i).dist_val < 0
         cycle_i(i).sign_dist = -1;
    else
         cycle_i(i).sign_dist = 0;
    end
    cycle_i(i).is_ghost_impact = NaN;
    
    %--- cycle i
    cycle_i(i).t = dfz.time(ind_pts_i); % abs time [t1:t2]
    cycle_i(i).ui = dz.complete_traject(ind_pts_i); % mocap position
    cycle_i(i).dui = d_dz(ind_pts_i); % vel    
    cycle_i(i).yi = dfz.complete_traject(ind_pts_i); % force sensor
    cycle_i(i).dyi = d_dfz(ind_pts_i); % yank
    
    cycle_i(i).z_b = z_b(ind_pts_i); % ball position
    cycle_i(i).z_p = z_p(ind_pts_i); % paddle position
    cycle_i(i).dzb_k = dz_b(ind_pts_i(2)); % ball velocity after impact
    
    cycle_i(i).ui_max = max(cycle_i(i).ui) - min(cycle_i(i).ui); % max input mag
    cycle_i(i).yi_max = max(cycle_i(i).yi) - min(cycle_i(i).yi); % max output mag
    cycle_i(i).dui_max = max(cycle_i(i).dui) - min(cycle_i(i).dui); % max der input mag
    cycle_i(i).dyi_max = max(cycle_i(i).dyi) - min(cycle_i(i).dyi); % max der output mag
    
    %--- display
    if display
        % position (ui)
        figure(100)
        if cycle_i(i).is_dist
            plot(cycle_i(i).t,cycle_i(i).ui,'r--')
        else
            if cycle_i(i).is_post_dist
                plot(cycle_i(i).t,cycle_i(i).ui,'--','color',[0.8500, 0.3250, 0.0980])
            else
                plot(cycle_i(i).t,cycle_i(i).ui,'g--')
            end
        end
        title("Position cycle #" + num2str(i))
        % force (yi)
        figure(101)
        if cycle_i(i).is_dist
            plot(cycle_i(i).t,cycle_i(i).yi,'r--')
        else
            if cycle_i(i).is_post_dist
                plot(cycle_i(i).t,cycle_i(i).yi,'--','color',[0.8500, 0.3250, 0.0980])
            else
                plot(cycle_i(i).t,cycle_i(i).yi,'g--')
            end
        end
        title("Force cycle #" + num2str(i))
        %     pause
    end
end

% post-processing: removing cycles that are too short
npts_tot = [cycle_i.npts]; 
cycle_iout = cycle_i; % cycle_i(~cellfun('isempty', cycle_i));
cycle_iout(npts_tot < 500) = []; % remove pathological cycles
