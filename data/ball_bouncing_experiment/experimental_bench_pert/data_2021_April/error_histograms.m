function error_histograms(errors,names)

% regroup data by user reference name
names_regr = names;
regrouped = [];
errors_n = errors;
for ii = 1:length(errors)
    if any(ii == regrouped)
        names_regr(ii) = [];
        continue
    end
    tmp = names;
    tmp(ii) = [""]; % mask the iith name to avoid seing itself as duplicate
    string_compare = strcmp(names(ii), tmp);
    idx = find(string_compare);
    if any(string_compare)
        for i_idx = idx
            regrouped = [regrouped, i_idx];
            errors_n{ii} = horzcat(errors{ii}, errors{i_idx});
        end
        errors_n{[idx]} = [];
    end
end
% remove empty cells
errors_n =  errors_n(~cellfun('isempty',errors_n));

%nb_cols = floor(length(errors_n)/5) + 1;
figure('DefaultAxesFontSize',16)
%tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact')
tiledlayout(3,1, 'TileSpacing', 'compact', 'Padding', 'compact')
for ii = 1:length(errors_n)
    nexttile
    try
        errs = vertcat(errors_n{ii}.data);
    catch
        errs = horzcat(errors_n{ii}.data);
    end
    prc_data = prctile(errs,[15,50,85]);
    hold on
    histogram(errs, 'BinWidth', 0.05)
    xline(prc_data(2),'k--','LineWidth',2)
    xline(prc_data(1),'r--','LineWidth',2)
    xline(prc_data(3),'r--','LineWidth',2)
end

end

