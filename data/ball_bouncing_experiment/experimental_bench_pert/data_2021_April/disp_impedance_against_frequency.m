function res = disp_impedance_against_frequency(cycles_norm, impedance, user_ref)

    NB_EXP = length(impedance);
    
    for exp_nb = 1:NB_EXP
        freq{exp_nb} = 1./[cycles_norm{exp_nb}([cycles_norm{exp_nb}.is_dist]).duration];
        stiff{exp_nb} = impedance{exp_nb}.xi(1,:);
        damp{exp_nb} = impedance{exp_nb}.xi(2,:);
        mass{exp_nb} = impedance{exp_nb}.xi(3,:);
        % delete last perturbation(s) that are outside the cyclic behaviour
        nb_pert_ignored = length(stiff{exp_nb}) - length(freq{exp_nb});
        if nb_pert_ignored > 0
            stiff{exp_nb}(end-nb_pert_ignored+1:end) = [];
            damp{exp_nb}(end-nb_pert_ignored+1:end) = [];
            mass{exp_nb}(end-nb_pert_ignored+1:end) = [];
            r2{exp_nb} = impedance{exp_nb}.r2_pos(1:length(freq{exp_nb}));
        end
        pert_type{exp_nb} = [cycles_norm{exp_nb}([cycles_norm{exp_nb}.is_dist]).type_dist];

        idx = r2{exp_nb}>0.5;
        for ii = 1:max(cellfun(@(x) max(x), pert_type))
            % r2>0.5 and classification
            pert_type_cln_idx(exp_nb, ii) = {(idx & (pert_type{exp_nb} == ii))};
            stiff_cln(exp_nb,ii) = {stiff{exp_nb}(pert_type_cln_idx{exp_nb, ii})};
            damp_cln(exp_nb,ii) = {damp{exp_nb}(pert_type_cln_idx{exp_nb, ii})};
            mass_cln(exp_nb,ii) = {mass{exp_nb}(pert_type_cln_idx{exp_nb, ii})};
            r2_cln(exp_nb,ii) = {r2{exp_nb}(pert_type_cln_idx{exp_nb, ii})};
            % classification with outliers
            stiff_class(exp_nb,ii) = {stiff{exp_nb}(pert_type{exp_nb} == ii)};
            % sMAD cleaning according to stiffnesses
            [stiff_cln{exp_nb,ii}, idxRM] = rmoutliers(stiff_cln{exp_nb,ii}, ...
                'ThresholdFactor', 5); % 5sMAD
            damp_cln{exp_nb,ii} = damp_cln{exp_nb,ii}(~idxRM);
            mass_cln{exp_nb,ii} = mass_cln{exp_nb,ii}(~idxRM);
            r2_cln{exp_nb,ii} = r2_cln{exp_nb,ii}(~idxRM);
            % counting outliers per class
            nb_outliers{exp_nb, ii} = sum(idxRM) + ...; % from sMAD
                sum((pert_type{exp_nb} == ii) & ~idx); % from r2>0.5
            tmp = freq{exp_nb}(pert_type_cln_idx{exp_nb, ii});
            freq_type_cln{exp_nb,ii} = tmp(~idxRM);
            tot_data{exp_nb,ii} = sum(pert_type{exp_nb} == ii);
        end
        
    end
    NB_EXP_REG = NB_EXP;
    regrouped = [];
    freq_type_cln_rg = freq_type_cln;
    stiff_cln_rg = stiff_cln;
    damp_cln_rg = damp_cln;
    mass_cln_rg = mass_cln;
    r2_cln_rg = r2_cln;
    nb_outliers_rg = nb_outliers;
    user_ref_regr = user_ref;
    if nargin > 2
        for exp_nb = 1:NB_EXP
            % skip if the session has already been regrouped with
            % another one
            if any(exp_nb == regrouped)
                user_ref_regr(exp_nb) = [];
                continue
            end
            tmp = user_ref;
            tmp(exp_nb) = [""];
            string_compare = strcmp(user_ref(exp_nb), tmp);
            idx = find(string_compare);
            if any(string_compare)
                NB_EXP_REG = NB_EXP_REG -1;
                regrouped = [regrouped, idx];
                for exp_reg = idx
                for ii = 1:max(cellfun(@(x) max(x), pert_type))
                    freq_type_cln_rg{exp_nb,ii} = [freq_type_cln{exp_nb,ii}, ...
                        freq_type_cln{exp_reg,ii}];
                    nb_outliers_rg{exp_nb, ii} = nb_outliers{exp_nb, ii} +  ...
                        nb_outliers{exp_reg, ii};
                    stiff_cln_rg{exp_nb,ii} = [stiff_cln{exp_nb,ii}, ...
                        stiff_cln{exp_reg,ii}];
                    damp_cln_rg{exp_nb,ii} = [damp_cln{exp_nb,ii}, ...
                        damp_cln{exp_reg,ii}];
                    mass_cln_rg{exp_nb,ii} = [mass_cln{exp_nb,ii}, ...
                        mass_cln{exp_reg,ii}];
                    r2_cln_rg{exp_nb,ii} = [r2_cln{exp_nb,ii}, ...
                        r2_cln{exp_reg,ii}];
                end
                    freq_type_cln_rg(exp_reg,:) = [];
                    stiff_cln_rg(exp_reg,:) = [];
                    damp_cln_rg(exp_reg,:) = [];
                    mass_cln_rg(exp_reg,:) = [];
                    r2_cln_rg(exp_reg,:) = [];
                    nb_outliers_rg(exp_reg,:) = [];
                end
            end
        end
    end
    
    classes = [];
    for ii = 1:max(cellfun(@(x) max(x), pert_type))
        r(ii) = mean([cycles_norm{1}([cycles_norm{1}.type_dist] == ii).ratio_dist]);
        classes = [classes, "c_" + num2str(ii)];
    end
    [~,idx_sort] = sort(r);
    [~,idx_sort_s] = sort(idx_sort);
    
    [sp,n] = numSubplots(NB_EXP_REG);
    colors = lines(NB_EXP_REG);
    marker_list = ['s','d','o'];
    
    figure('DefaultAxesFontSize',16)
    for exp_nb = 1:NB_EXP_REG
        % subplot(sp(1),sp(2),exp_nb)
        hold on
%         p(exp_nb) = plot(freq{exp_nb}, stiff{exp_nb}, marker_list(exp_nb), 'Color', colors(exp_nb,:));
        for ii = 1:max(cellfun(@(x) max(x), pert_type))
%             p(exp_nb,ii) = plot(freq_type_cln{exp_nb,ii}, stiff_cln{exp_nb,ii},...
%                 marker_list(ii), 'Color', colors(exp_nb,:), 'MarkerFaceColor',...
%                 colors(exp_nb,:));
            p(exp_nb,ii) = scatter(freq_type_cln_rg{exp_nb,ii}, stiff_cln_rg{exp_nb,ii},...
                marker_list(ii), 'MarkerFaceColor', colors(exp_nb,:), ...
                'MarkerEdgeColor', colors(exp_nb,:));
            %p(exp_nb,ii).Color(4) = 0.2;
            p(exp_nb,ii).MarkerFaceAlpha = .2;
        end
    end
    
    %legend([p(1,idx_sort(1)), p(1,idx_sort(2)), p(1,idx_sort(3))], classes)

   
    %figure
    for exp_nb = 1:NB_EXP_REG
        % subplot(sp(1),sp(2),exp_nb)
        hold on
%         p(exp_nb) = plot(freq{exp_nb}, stiff{exp_nb}, marker_list(exp_nb), 'Color', colors(exp_nb,:));
        for ii = 1:max(cellfun(@(x) max(x), pert_type))
            qrtileX = prctile(freq_type_cln_rg{exp_nb,ii},[25,50,75]);
            qrtileY = prctile(stiff_cln_rg{exp_nb,ii},[25,50,75]);
            eb = errorbar(qrtileX(2),qrtileY(2),abs(qrtileY(1)-qrtileY(2)),...
                abs(qrtileY(3)-qrtileY(2)),abs(qrtileX(1)-qrtileX(2)),...
                abs(qrtileX(3)-qrtileX(2)),marker_list(ii), 'Color', ...
                colors(exp_nb,:), 'MarkerSize', 15,...%, 'MarkerFaceColor', colors(exp_nb,:), 
                'LineWidth', 2);
            alpha = 0.65;   
            % Set transparency (undocumented)
            set([eb.Bar, eb.Line], 'ColorType', 'truecoloralpha', ...
                'ColorData', [eb.Line.ColorData(1:3); 255*alpha]);
%             currentunits = get(gca,'Units');
%             set(gca, 'Units', 'Points');
%             axpos = get(gca,'Position');
%             set(gca, 'Units', currentunits);
%             markerWidth = 20/diff(xlim)*axpos(3); % Calculate Marker width in points
%             set(sc, 'SizeData', markerWidth^2)
            sc = scatter(qrtileX(2),qrtileY(2),240,marker_list(ii),...
                'MarkerFaceColor', colors(exp_nb,:), ...
                'MarkerEdgeColor', colors(exp_nb,:));
            sc.MarkerFaceAlpha = alpha/2;

        end
    end
%     if nargin > 2
%         legend(p(1,:), classes)
%     end
    legend(p(1,idx_sort_s), classes)
    
    xlabel('Frequency (Hz)')
    ylabel('Stiffness (N.m^{-1})')
    
    for i = 1:length(idx_sort_s)
        res.class(1,i) = classes(idx_sort_s(i));
    end
    
    res.frequencies = freq_type_cln_rg;
    res.stiffness = stiff_cln_rg;
    res.damping = damp_cln_rg;
    res.mass = mass_cln_rg;
    res.r2 = r2_cln_rg;
    res.outliers_nb = nb_outliers_rg;
    res.headers = user_ref_regr;

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
    
    