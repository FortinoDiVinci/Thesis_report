function cycles_regr = regroupCycles(cycles)
%

cycles_regr = cycles;
for ii = 1:length(cycles)
    names(ii) = string(cycles{ii}(1).user);
end
names_regr = names;
regrouped = [];
for ii = 1:length(cycles)
    if any(ii == regrouped)
        names_regr(ii) = "";
        continue
    end
    tmp = names;
    tmp(ii) = [""]; % mask the iith name to avoid seing itself as duplicate
    string_compare = strcmp(names(ii), tmp);
    idx = find(string_compare);
    it = 1;
    c = cell(length(cycles_regr{ii}),1);
    c(:) = {it};
    [cycles_regr{ii}.exp_it] = c{:};
    if any(string_compare)
        for i_idx = idx
            regrouped = [regrouped, i_idx];
            it = it + 1;
            c = cell(length(cycles{i_idx}),1);
            c(:) = {it};
            [cycles{i_idx}.exp_it] = c{:};
            cycles_regr{ii} = horzcat(cycles_regr{ii}, cycles{i_idx});
            cycles_regr{[i_idx]} = [];
        end
    end
end
names_regr = names_regr(~(names_regr==""));
cycles_regr =  cycles_regr(~cellfun('isempty',cycles_regr));

end

