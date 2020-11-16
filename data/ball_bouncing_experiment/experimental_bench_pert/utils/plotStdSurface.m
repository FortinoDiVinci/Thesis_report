function plotStdSurface(mean_vec, std_vec, t_vec, color, std_coeff)
% plot a surface showing the standard deviation
    
    if nargin < 2
        error('Not enough inputs provided..');
    elseif nargin < 3
        t_vec = (1:1:length(mean_vec));
        color = 'k';
        std_coeff = 1;
    elseif nargin < 4
        color = 'k';
        std_coeff = 1;
    elseif nargin < 5
        std_coeff = 1;
    end

    if isempty(t_vec)
        t_vec = (1:1:length(mean_vec));
    end
    
    curve1 = mean_vec + std_vec.*std_coeff;
    curve2 = mean_vec - std_vec.*std_coeff; 
    
    %fill([t_vec fliplr(t_vec)],[curve1' fliplr(curve2')], color, 'FaceAlpha', 0.2, 'EdgeColor', 'none')
    fill([t_vec fliplr(t_vec)],[curve1' fliplr(curve2')], color, 'FaceAlpha', 0.2, 'EdgeColor', color)
end

