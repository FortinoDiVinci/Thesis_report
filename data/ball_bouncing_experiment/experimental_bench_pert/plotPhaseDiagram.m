function fig = plotPhaseDiagram(cycles,varargin)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    % Default values
    PLOT_PERTURBATIONS = 0;
    PLOT_IMPACTS = 0;
    idx_imp = [];
    idx_pert = [];
    x_lim = [-8, 8];
    figure_name = "Phase Diagram";

    tmp_pos = {cycles.position};
    tmp_vel = {cycles.velocity};
    position = tmp_pos{:};
    velocity = tmp_vel{:};
    clear tmp_pos temp_vel;
    
    if ~isempty(varargin)
        for ii = 1:2:length(varargin)
            switch(varargin{ii})
                case 'PositionCentering'
                    is_position_avg = varargin{ii+1};
                case 'PlotImpacts'
                    idx_imp = varargin{ii+1};
                case 'PlotPerturbations'
                    if varargin{ii+1} == 1 || strcmpi(varargin{ii+1},"y") 
                        PLOT_PERTURBATIONS = 1;
                    %idx_pert = varargin{ii+1};
                    end
                case 'xlim'
                    x_lim = varargin{ii+1};
                case 'FigureName' 
                    figure_name = varargin{ii+1};
                otherwise
                    %
                    warning("Unknown argument " + string(varargin{ii}))
            end
        end
    end

    if PLOT_PERTURBATIONS
        idx_pert = [idx_pert, cycles.pert_idx];
    end
        
    if is_position_avg
        fc = 0.1; % cut off frequency
        [b2,a2] = butter(2,fc/(1/(2*dt)),'low'); 
        position_avg = filtfilt(b2,a2,zh);
    else
        position_avg = zeros(size(position));
    end
    
    fig = figure();
    p1 = plot(position - position_avg, velocity);
    p1.Color(4) = 0.5;
    hold on
    p_imp = plot(zh(idx_imp)-z_avg(idx_imp), dzh(idx_imp), '.r', 'MarkerSize',30);
    p_per = plot(zh(idx_pert)-z_avg(idx_pert), dzh(idx_pert), '.m', 'MarkerSize',30);
    p_imp.Color(4) = 0.8;
    p_per.Color(4) = 0.8;
    xlim(x_lim)
    %title('Phase diagram ' + string(i))
    title(figure_name)

    
    
end

