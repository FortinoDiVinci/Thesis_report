function cycle_iout = cycle_extraction(ind_i_red, dt, dfz, dz, z_b, z_p, display)

% cycle_iout = decouper_cycles(n_essai,ind_i_red,ind,dist,ts,t,z,fz,z_b,z_p,afficher)
% -- pour accéder à un champ de la structure à travers les cycles : 
% -- C = max(cellfun(@(S)(S.npts),cycle_i)) % exemple de calcul du nb de
% -- pts max parmi tous les cycles

cycle_i = {};
cycle_i(1).is_post_dist = 0;

d_dz = Iu_diffcent(dz.complete_traject, dz.time);
d_dfz = Iu_diffcent(dfz.complete_traject, dfz.time);

for i = 1:length(ind_i_red) - 1
    %--- opérations sur temps et durée
    ind_pts_i = [ind_i_red(i):ind_i_red(i+1) - 1]'; % indices absolus du cycle
    
    cycle_i(i).npts = length(ind_pts_i); % nb de pts dans le cycle
    cycle_i(i).t_red = [0:cycle_i(i).npts-1]'*dt; % temps de cycle [0:tend]
    cycle_i(i).duration = max(cycle_i(i).t_red); % durée du cycle
    
    %--- opérations sur indices et détection dist
   
    [ind_dist_i,~,IB] = intersect(ind_pts_i, dfz.pert_ind); % indices absolus début et fin pert du cycle
    cycle_i(i).ind = ind_dist_i - ind_i_red(i); % indices relatifs début et fin pert du cycle
    cycle_i(i).glob_ind = ind_dist_i; % indices global
    cycle_i(i).is_dist = ~isempty(ind_dist_i); % cycle perturbé ou non
    if i > 1
        cycle_i(i).is_post_dist = (cycle_i(i-1).is_dist) == 1; % cycle précédent perturbé
    end
    
    
    if cycle_i(i).is_dist
        cycle_i(i).dist_val = dfz.pert_val(IB(1));
        cycle_i(i).ratio_dist = cycle_i(i).ind(1)./cycle_i(i).npts; % instant normalisé de perturbation dans le cycle
    else
        cycle_i(i).dist_val = 0;
        cycle_i(i).ratio_dist = -1;
    end
    
    if cycle_i(i).dist_val > 0
        cycle_i(i).is_pos_dist = 1;
    elseif cycle_i(i).dist_val < 0
         cycle_i(i).is_pos_dist = -1;
    else
         cycle_i(i).is_pos_dist = 0;
    end
    
    %--- opérations sur signaux dans le cycle i
    cycle_i(i).t = dfz.time(ind_pts_i); % temps absolu [t1:t2]
    cycle_i(i).ui = dz.complete_traject(ind_pts_i); % pos mocap  
    cycle_i(i).dui = d_dz(ind_pts_i); % vel    
    cycle_i(i).yi = dfz.complete_traject(ind_pts_i); % force capteur
    cycle_i(i).dyi = d_dfz(ind_pts_i); % yank
    
    cycle_i(i).z_b = z_b(ind_pts_i); % pos balle 
    cycle_i(i).z_p = z_p(ind_pts_i); % pos raquette 
    
    cycle_i(i).ui_max = max(cycle_i(i).ui) - min(cycle_i(i).ui); % amp max entrée
    cycle_i(i).yi_max = max(cycle_i(i).yi) - min(cycle_i(i).yi); % amp max sortie
    cycle_i(i).dui_max = max(cycle_i(i).dui) - min(cycle_i(i).dui); % amp max derv entrée
    cycle_i(i).dyi_max = max(cycle_i(i).dyi) - min(cycle_i(i).dyi); % amp max derv sortie
    
    %--- affichage
    if display
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
        %     pause
    end
end

% post-traitement pour éliminer les cycles erronés (longueur trop faible)

npts_tot = [cycle_i.npts]; %cellfun(@(S)(S.npts), cycle_i);
pathological_cycles = find(npts_tot < 500);
% for k = 1:length(pathological_cycles)
%     cycle_i(pathological_cycles(k)) = [];
% end
cycle_iout = cycle_i; % cycle_i(~cellfun('isempty', cycle_i));
cycle_iout(pathological_cycles) = [];
