function disp_impedance_per_cycle(cycles_norm, impedance, user_ref)

    NB_EXP = length(impedance);

    for exp_nb = 1:NB_EXP
        ratio{exp_nb} = [cycles_norm{exp_nb}([cycles_norm{exp_nb}.is_dist]).ratio_dist];
        stiff{exp_nb} = impedance{exp_nb}.xi(1,:);
        % delete last perturbation(s) that are outside the cyclic behaviour
        nb_pert_ignored = length(stiff{exp_nb}) - length(ratio{exp_nb});
        if nb_pert_ignored > 0
            stiff{exp_nb}(end-nb_pert_ignored+1:end) = [];
        end
        r2{exp_nb} = impedance{exp_nb}.r2_pos;
        pert_type{exp_nb} = [cycles_norm{exp_nb}([cycles_norm{exp_nb}.is_dist]).type_dist];
    end
    
%     r2_max=max(cellfun(@(x) max(x), r2));
%     r2_min=min(cellfun(@(x) min(x), r2));
    
    [p,n]=numSubplots(NB_EXP);
    colors = lines(NB_EXP);
    
    figure
    hold on
    for exp_nb = 1:NB_EXP
        %subplot(p(1),p(2),exp_nb)
        p(exp_nb) = plot(ratio{exp_nb}, stiff{exp_nb}, '*', 'Color', colors(exp_nb,:))
        for ii = 1:max(cellfun(@(x) max(x), pert_type))
            idx1 = [pert_type{exp_nb} == ii];
            plot(median(ratio{exp_nb}(idx1)), median(stiff{exp_nb}(idx1)), ...
                'o', 'Color', colors(exp_nb,:), 'Markersize', 10);
        end
    end
    if nargin > 2
        legend(p, user_ref)
    end
    ylim([0, 1000])
    xlim([0,0.9])
    



function [p,n]=numSubplots(n)
% function [p,n]=numSubplots(n)
%
% Purpose
% Calculate how many rows and columns of sub-plots are needed to
% neatly display n subplots. 
%
% Inputs
% n - the desired number of subplots.     
%  
% Outputs
% p - a vector length 2 defining the number of rows and number of
%     columns required to show n plots.     
% [ n - the current number of subplots. This output is used only by
%       this function for a recursive call.]
%
%
%
% Example: neatly lay out 13 sub-plots
% >> p=numSubplots(13)
% p = 
%     3   5
% for i=1:13; subplot(p(1),p(2),i), pcolor(rand(10)), end 
%
%
% Rob Campbell - January 2010
   
    
while isprime(n) & n>4, 
    n=n+1;
end
p=factor(n);
if length(p)==1
    p=[1,p];
    return
end
while length(p)>2
    if length(p)>=4
        p(1)=p(1)*p(end-1);
        p(2)=p(2)*p(end);
        p(end-1:end)=[];
    else
        p(1)=p(1)*p(2);
        p(2)=[];
    end    
    p=sort(p);
end
%Reformat if the column/row ratio is too large: we want a roughly
%square design 
while p(2)/p(1)>2.5
    N=n+1;
    [p,n]=numSubplots(N); %Recursive!
end