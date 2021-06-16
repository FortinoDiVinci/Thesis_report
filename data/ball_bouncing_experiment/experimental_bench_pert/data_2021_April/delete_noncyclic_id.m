function impedance_cln = delete_noncyclic_id(cycles, impedance)
% delete_noncyclic_pert
% deletes last perturbation(s) that are outside the cyclic behaviour

NB_EXP = length(impedance);
impedance_cln = impedance;

for exp_nb = 1:NB_EXP
    nb_pert_cyc = sum([cycles{exp_nb}.is_dist]);
    nb_pert_tot = impedance{exp_nb}.nb_id;
    if nb_pert_cyc < nb_pert_tot
        impedance_cln{exp_nb}.nb_id = nb_pert_cyc;
        impedance_cln{exp_nb}.phi = impedance_cln{exp_nb}.phi(:,:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.y = impedance_cln{exp_nb}.y(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.xi = impedance_cln{exp_nb}.xi(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.rec_y = impedance_cln{exp_nb}.rec_y(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.rec_err = impedance_cln{exp_nb}.rec_err(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.rec_err_norm = impedance_cln{exp_nb}.rec_err_norm(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.r_2 = impedance_cln{exp_nb}.r_2(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.rel_std = impedance_cln{exp_nb}.rel_std(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.rmse = impedance_cln{exp_nb}.rmse(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.nrmse = impedance_cln{exp_nb}.nrmse(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.arx_id = impedance_cln{exp_nb}.arx_id(1:nb_pert_cyc);
        impedance_cln{exp_nb}.rec_pos = impedance_cln{exp_nb}.rec_pos(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.rec_pos_err = impedance_cln{exp_nb}.rec_pos_err(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.nrmse_pos = impedance_cln{exp_nb}.nrmse_pos(:,1:nb_pert_cyc);
        impedance_cln{exp_nb}.r2_pos = impedance_cln{exp_nb}.r2_pos(:,1:nb_pert_cyc);
    end
end


end

