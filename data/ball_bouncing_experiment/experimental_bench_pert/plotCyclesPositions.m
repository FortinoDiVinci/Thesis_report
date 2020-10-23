function plotCyclesPositions(cycles, cycle_mean, cycle_std)
% TODO
%   Detailed explanation goes here
    figure
    hold on
    for c_n = cycles
        switch c_n.pert_dir
            case 0
                plot(c_n.time, c_n.position, 'color', [0, 0.4470, 0.7410])
            case 1
                plot(c_n.time, c_n.position, 'color', [0.4660, 0.6740, 0.1880])
            case -1
                plot(c_n.time, c_n.position, 'color', [0.6350, 0.0780, 0.1840])
        end
    end
    
    if nargins > 1
        plot(cycle_mean.time, cycle_mean.position, 'k', 'linewidth', 2)
        
    end
    if nargins > 2
        fill([cycle_mean.time fliplr(cycle_mean.time)], ...
            [(cycle_mean.position+cycle_std.position)' ...
            fliplr((cycle_mean.position-cycle_std.position)')], ...
            [0.25, 0.25, 0.25], 'FaceAlpha', 0.3,'linestyle','-','edgecolor',[0.25, 0.25, 0.25])
    end
  
end

