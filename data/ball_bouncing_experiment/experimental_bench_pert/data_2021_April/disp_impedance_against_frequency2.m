function disp_impedance_against_frequency2(data_sorted, users_ref, class_order, exp_order)
% disp_impedance_against_frequency2
%  
% data_sorted should be a struct array with at least those fields:
% - class (string)
% - user (string)
% - frequencies (1xn double array)
% - stiffness (1xn ouble array)
% - r2 (1xn double array)
% user_ref should be a string array

%% regroup impedance data according to users_ref
data_sorted_rg = data_sorted;
users_ref_regr = users_ref;
regrouped = [];
for ii = 1:length(users_ref)
    if any(ii == regrouped)
        users_ref_regr(ii) = "";
        continue
    end
    tmp = users_ref_regr;
    tmp(ii) = "";
    idx = find(strcmp(tmp, users_ref_regr(ii)));
    if ~isempty(idx)
        for i_idx = idx
            regrouped = [regrouped, i_idx];
            % iterates throught the different classes
            for ci = 1:size(data_sorted, 2)
                % concatenate structure fields according to their nature
                for field_i = fieldnames(data_sorted_rg)'
                    fi = field_i{1};
                    if strcmp(fi, 'class')
                        continue
                    elseif strcmp(fi, 'outliers_nb')
                        data_sorted_rg(ii,ci).(fi) = data_sorted_rg(ii,ci).(fi) +...
                        data_sorted_rg(i_idx,ci).(fi);
                    else
                        data_sorted_rg(ii,ci).(fi) = [data_sorted_rg(ii,ci).(fi), ...
                            data_sorted_rg(i_idx,ci).(fi)];
                    end
                end
            end
        end
    end
    % set the expertise lvl
    for ci = 1:size(data_sorted, 2)
        data_sorted_rg(ii,ci).expertise = users_ref(ii);
    end
    
end
data_sorted_rg(users_ref_regr == "",:) = [];
users_ref_regr(users_ref_regr == "") = [];
data_sorted_rg_ns = data_sorted_rg;

%% sort data in the specified order 
% update class order
if nargin > 2
    class_current_order = [];
    for ci = 1:size(data_sorted_rg,2)
        class_current_order = [class_current_order, data_sorted_rg_ns(1,ci).class];
    end
    for ci = 1:size(data_sorted_rg,2)
        if strcmp(class_current_order(ci), class_order(ci))
            continue
        else
            %ci_cor = find();
            data_sorted_rg(:,ci) = data_sorted_rg_ns(:, class_current_order == class_order(ci));
        end
    end
    clear class_current_order
end
% update user ref order
if nargin > 3
    data_sorted_rg_ns = data_sorted_rg;
    exp_current_order = [];
    for ei = 1:size(data_sorted_rg,1)
        exp_current_order = [exp_current_order, data_sorted_rg_ns(ei,1).expertise];
    end
    for ei = 1:size(data_sorted_rg,1)
        if strcmp(exp_current_order(ei), exp_order(ei))
            continue
        else
            %ci_cor = find();
            data_sorted_rg(ei,:) = data_sorted_rg_ns(exp_current_order == exp_order(ei), :);
        end
    end
    clear exp_current_order
end

%% plot stiffness against frequencies
NB_USER = length(users_ref_regr);
colors = lines(NB_USER);
marker_list = ['s','d','o']; % for 3 classes

figure('DefaultAxesFontSize',28)
% plot all estimated stiffness in smaller markers (with transparency)
for user_i = 1:NB_USER
    hold on
    for ci = 1:size(data_sorted_rg, 2)
        p(user_i,ci) = scatter(data_sorted_rg(user_i,ci).frequencies, ...
            data_sorted_rg(user_i,ci).stiffness, marker_list(ci), ...
            'MarkerFaceColor', colors(user_i,:), 'MarkerEdgeColor', ...
            colors(user_i,:), 'SizeData', 200);
        p(user_i,ci).MarkerFaceAlpha = .2;
    end
end
% bigger markers with error bar
for user_i = 1:NB_USER
    hold on
    for ci = 1:size(data_sorted_rg, 2)
        qrtileX = prctile(data_sorted_rg(user_i,ci).frequencies,[25,50,75]);
        qrtileY = prctile(data_sorted_rg(user_i,ci).stiffness,[25,50,75]);
        eb = errorbar(qrtileX(2),qrtileY(2),abs(qrtileY(1)-qrtileY(2)),...
            abs(qrtileY(3)-qrtileY(2)),abs(qrtileX(1)-qrtileX(2)),...
            abs(qrtileX(3)-qrtileX(2)),marker_list(ci), 'Color', ...
            colors(user_i,:), 'MarkerSize', 50, 'LineWidth', 4);
        alpha = 0.65;   
        % Set transparency (undocumented) for bars
        set([eb.Bar, eb.Line], 'ColorType', 'truecoloralpha', ...
            'ColorData', [eb.Line.ColorData(1:3); 255*alpha]);
        sc = scatter(qrtileX(2),qrtileY(2),240,marker_list(ci),...
            'MarkerFaceColor', colors(user_i,:), 'MarkerEdgeColor', ...
            colors(user_i,:), 'SizeData', 2500);
        sc.MarkerFaceAlpha = alpha/2;
    end
end
class_names = [];
for ci = 1:size(data_sorted_rg, 2)
    class_names = [class_names, data_sorted_rg(1,ci).class];
end
%legend(p(1,:), [data_sorted_rg(1,1).class, data_sorted_rg(1,2).class, data_sorted_rg(1,3).class])
% l = legend(p(1,:), class_names);
[hLg, icons] = legend(p(1,:), class_names);
xlabel('Frequency (Hz)')
ylabel('Stiffness (N.m^{-1})')

icons = findobj(icons,'Type','patch');
icons = findobj(icons,'Marker','none','-xor');
set(icons(:),'MarkerSize',25);
set(icons(:),'MarkerFaceColor',[1,1,1]);
set(icons(:),'LineWidth', 2);

end

