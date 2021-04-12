function [] = disp_ghosted_norm(cycles, exp_class)

    cycle_ghost = cell(max(exp_class),1);
    cycle_nghost = cell(max(exp_class),1);
    new_cycles = cycles;
    % sort ghosted data according to the experiment
    for k = 1:length(new_cycles)
        idx_is_ghost = [new_cycles{k}.is_ghost_impact] == 1; 
        idx_is_nghost = [new_cycles{k}.is_ghost_impact] == 0;
        tmp = num2cell(k.*ones(size(new_cycles{k})));
        [new_cycles{k}(:).exp_count] = deal(tmp{:});
        cycle_ghost{exp_class(k)} = [cycle_ghost{exp_class(k)}, new_cycles{k}(idx_is_ghost)];                
        cycle_nghost{exp_class(k)} = [cycle_nghost{exp_class(k)}, new_cycles{k}(idx_is_nghost)];
    end

    for i = 1:length(cycle_ghost)
        force_avg_g{i} = mean(vertcat(cycle_ghost{i}.yi_interp));
        force_std_g{i} = std(vertcat(cycle_ghost{i}.yi_interp));
        force_avg_ng{i} = mean(vertcat(cycle_nghost{i}.yi_interp));
        force_std_ng{i} = std(vertcat(cycle_nghost{i}.yi_interp));
    end
    
    titles_list = ["1) 10% ghosted", "2) 90% ghosted", ...
        "3) 10% ghosted & stiff", "4) 90% ghosted & stiff"];
    %%    
    % compare ghosted with non ghosted for each exp
    figure('DefaultAxesFontSize',13)
    for i = 1:length(cycle_ghost)
        subplot(2,2,i)
        hold on
        p1 = plot(cycle_ghost{i}(1).t_com, force_avg_g{i}, '--', ...
            'Color', [0, 0.4470, 0.7410], 'Linewidth', 1.5);
        plotStdSurface(force_avg_g{i}', force_std_g{i}', cycles{i}(1).t_com, [0, 0.4470, 0.7410], 3)
        plotStdSurface(force_avg_g{i}', force_std_g{i}', cycles{i}(1).t_com, [0, 0.4470, 0.7410], 1)
        
        p2 = plot(cycle_ghost{i}(1).t_com, force_avg_ng{i}, '--', ...
            'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5);
        plotStdSurface(force_avg_ng{i}', force_std_ng{i}', cycles{i}(1).t_com, [0.8500, 0.3250, 0.0980], 3)
        plotStdSurface(force_avg_ng{i}', force_std_ng{i}', cycles{i}(1).t_com, [0.8500, 0.3250, 0.0980], 1)
        
        title(titles_list(i))
        ylim([min(force_avg_ng{i} - 3.*force_std_ng{i}), max(force_avg_ng{i} + 3.*force_std_ng{i})])
        if i == 1
            legend([p1, p2], {'ghosted','w/ impact'})
        end
    end
    %%
    % compare 10% ghosted with 90%
    figure('DefaultAxesFontSize',13)
    for i = 1:2:length(cycle_ghost)
        subplot(2,2,i)
        hold on
        p1 = plot(cycle_ghost{i}(1).t_com, force_avg_g{i}, '--', ...
            'Color', [0, 0.4470, 0.7410], 'Linewidth', 1.5);
        plotStdSurface(force_avg_g{i}', force_std_g{i}', cycles{i}(1).t_com, [0, 0.4470, 0.7410], 3)
        plotStdSurface(force_avg_g{i}', force_std_g{i}', cycles{i}(1).t_com, [0, 0.4470, 0.7410], 1)
        
        p2 = plot(cycle_ghost{i}(1).t_com, force_avg_g{i+1}, '--', ...
            'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5);
        plotStdSurface(force_avg_g{i+1}', force_std_g{i+1}', cycles{i}(1).t_com, [0.8500, 0.3250, 0.0980], 3)
        plotStdSurface(force_avg_g{i+1}', force_std_g{i+1}', cycles{i}(1).t_com, [0.8500, 0.3250, 0.0980], 1)
        
        cond = "";
        if i == 3; cond = " stiff"; end 
        title("Ghosted" + cond)
        ylim([min(force_avg_g{i} - 3.*force_std_g{i}), max(force_avg_g{i} + 3.*force_std_g{i})])
        if i == 1
            legend([p1, p2], {'10% gh.','90% gh.'})
        end
    end
    
    for i = 1:2:length(cycle_ghost)
        subplot(2,2,i+1)
        hold on
        p1 = plot(cycle_ghost{i}(1).t_com, force_avg_ng{i}, '--', ...
            'Color', [0, 0.4470, 0.7410], 'Linewidth', 1.5);
        plotStdSurface(force_avg_ng{i}', force_std_ng{i}', cycles{i}(1).t_com, [0, 0.4470, 0.7410], 3)
        plotStdSurface(force_avg_ng{i}', force_std_ng{i}', cycles{i}(1).t_com, [0, 0.4470, 0.7410], 1)
        
        p2 = plot(cycle_ghost{i}(1).t_com, force_avg_ng{i+1}, '--', ...
            'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5);
        plotStdSurface(force_avg_ng{i+1}', force_std_ng{i+1}', cycles{i}(1).t_com, [0.8500, 0.3250, 0.0980], 3)
        plotStdSurface(force_avg_ng{i+1}', force_std_ng{i+1}', cycles{i}(1).t_com, [0.8500, 0.3250, 0.0980], 1)
        
        cond = "";
        if i == 3; cond = " stiff"; end
        title("Non ghosted" + cond)
        ylim([min(force_avg_ng{i} - 3.*force_std_ng{i}), max(force_avg_ng{i} + 3.*force_std_ng{i})])
        if i == 1
            legend([p1, p2], {'10% gh.','90% gh.'})
        end
    end
    
    %%
    
    for i = length(force_avg_g):-1:1
        force_avg_tot(2*i-1,:) = force_avg_g{i};
        force_avg_tot(2*i,:) = force_avg_ng{i};
        force_std_tot(2*i-1,:) = force_std_g{i};
        force_std_tot(2*i,:) = force_std_ng{i};
    end
    n = length(cycle_ghost)*2;
%     figure('DefaultAxesFontSize',12)
%     for i = 1:n
%         for j = i+1:n
%             idx = i*(n-1) - i*(i-1)/2 - (n-j);
%             r1 = rem(i+1,2);
%             q1 = (i+1-r1)/2;
%             r2 = rem(j+1,2);
%             q2 = (j+1-r2)/2;
%             
%             str = "";
%             if r1; str = "n"; end
%             s1 = "Exp n°" + num2str(q1) + " " + str +  "ghost";
%             str = "";
%             if r2; str = "n"; end
%             s2 = "n°" + num2str(q2) + " " + str +  "ghost";
%             
%             diff_var = force_avg_tot(i,:) + force_std_tot(i,:) - (...
%                 force_avg_tot(j,:) + force_std_tot(j,:));
%             subplot(7,4, idx)
%             plot(diff_var)
%             title(s1 + " vs " + s2)
%         end
%     end

    
end

