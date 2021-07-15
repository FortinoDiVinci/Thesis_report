function cycles_clean = clearOutliers(cycles)
%
cycles_clean = cycles;

for user_i = 1:length(cycles)
    %%% delete non perturbed cycles
    %%cycles_clean{user_i}([cycles_clean{user_i}.is_dist == 0]) = [];    
    idx_id = ([cycles_clean{user_i}.is_dist] == 1);    
    nb_type = max([cycles{user_i}(idx_id).type_dist]);
    % init is outlier
    c = cell(length(cycles{user_i}),1);
    c(:) = {NaN};
    [cycles_clean{user_i}(:).is_outl] = c{:};
    % set to 0 all identification
    c = cell(sum(~isnan([cycles{user_i}.R2])),1);
    c(:) = {0};
    [cycles_clean{user_i}(~isnan([cycles{user_i}.R2])).is_outl] = c{:};
    % set to 1 all identification with R^2 score below 50%
    c = cell(sum([cycles{user_i}.R2] <= 0.5),1);
    c(:) = {1};
    [cycles_clean{user_i}([cycles{user_i}.R2] <= 0.5).is_outl] = c{:};
    
    % setting outlier flag for data outside 3 sMAD for the stiffness
    for type_i = 1:nb_type
        % sorting data by perturbation phase and direction
        idx1 = find([cycles{user_i}.type_dist] == type_i & ...
            [cycles{user_i}.sign_dist] == 1);
        idx2 = find([cycles{user_i}.type_dist] == type_i & ...
            [cycles{user_i}.sign_dist] == -1);
        % outliers indexes
        [~, idx1_rm] = rmoutliers([cycles{user_i}(idx1).K]); % , 'ThresholdFactor', 3
        [~, idx2_rm] = rmoutliers([cycles{user_i}(idx2).K]);
        % setting the flag for the outliers detected
        c = cell(sum(idx1_rm),1);
        c(:) = {1};
        [cycles_clean{user_i}(idx1(idx1_rm)).is_outl] = c{:};
        c = cell(sum(idx2_rm),1);
        c(:) = {1};
        [cycles_clean{user_i}(idx2(idx2_rm)).is_outl] = c{:};        
    end
    
end


end

