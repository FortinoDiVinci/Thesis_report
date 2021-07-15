function cycles_imp = addImpedance(cycles, impedance)
% add impedance data to cycle struct
%%% improvment
% TODO: check that every perturbed cycle pairs with the timing of an
% impedance identification ??

if length(cycles) ~= length(impedance)
    error("The cyclic struct array and impedance object array should have the same size")
end

cycles_imp = cycles;

for ii = 1:length(cycles)
    
    cyc = cycles{ii};
    
    % only cycles with perturbations are used here
    idx = [cyc.is_dist];
    
    % first perturbation time frame
    t1 = cyc(find(idx == 1, 1, 'first')).t(1);
    t2 = cyc(find(idx == 1, 1, 'first')).t(end);
    % last perturbation time frame
    te1 = cyc(find(idx == 1, 1, 'last')).t(1);
    te2 = cyc(find(idx == 1, 1, 'last')).t(end);
    
    j = 0; 
    % skip first perturbations that might have occured before cyclic data
    while impedance{ii}.t_pert(j+1) < t1
        % first perturbations occured before cyclic data (skip it)
        j = j + 1;
        if j > length(impedance{ii}.t_pert)
            error("TODO: no impedance data fit cyclic data...");
        end
    end
    k = 0;
    % skip last perturbations that might have occured after cyclic data
    if (impedance{ii}.nb_id - j) > sum(idx)
        k = impedance{ii}.nb_id - j - sum(idx);
    end
    
    c = cell(length(cyc), 1);
    c(:) = {NaN};
    [cycles_imp{ii}(:).K] = c{:};
    [cycles_imp{ii}(:).B] = c{:};
    [cycles_imp{ii}(:).M] = c{:};
    [cycles_imp{ii}(:).R2] = c{:};
    [cycles_imp{ii}(:).ui_r] = c{:};
    [cycles_imp{ii}(:).t_r] = c{:};
    
    idx_perts = find(idx);
    for jj = 1:length(idx_perts)
        % feed corresponding impedance data
        cycles_imp{ii}(idx_perts(jj)).K = impedance{ii}.xi(1, jj + j);
        cycles_imp{ii}(idx_perts(jj)).B = impedance{ii}.xi(2, jj + j);
        cycles_imp{ii}(idx_perts(jj)).M = impedance{ii}.xi(3, jj + j);
        cycles_imp{ii}(idx_perts(jj)).R2 = impedance{ii}.r2_pos(jj + j);
        cycles_imp{ii}(idx_perts(jj)).ui_r = impedance{ii}.rec_pos(:, jj + j);
        cycles_imp{ii}(idx_perts(jj)).t_r = impedance{ii}.t(:, jj + j);
    end

end


end

