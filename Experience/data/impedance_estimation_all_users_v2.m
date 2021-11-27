% for the final experiment, data was splited for memory purpose
clear all

nb_param = 3; % K, B, M
wndw_imp_eval = 150;
dt = 1e-3;

load('users_fz/experiments_list.mat')

users_list = unique(string(vertcat(exp_parameters.user)))';
k = length(users_list);

for user_name = users_list
    
    load("users_fz/exp_delta_fz_2021_" + user_name + ".mat", "delta_fz");
    load("users_pz/exp_delta_z_2021_" + user_name + ".mat", "delta_z");
    
    for ii = 1:length(delta_fz)
        impedance(ii) = IMPEDANCE_DATA(nb_param, delta_fz{ii}.nb_traject,...
        wndw_imp_eval, delta_z{ii}.header);
        impedance(ii).init_phi_arx(delta_z{ii}.diff_traject(1:wndw_imp_eval,:));
        impedance(ii).init_y(delta_fz{ii}.diff_traject(1:wndw_imp_eval,:));
        impedance(ii).init_t(delta_fz{ii}.t_traject(3,:));
        impedance(ii).arx_quick();
        impedance(ii).causalSim(dt, 'NulInitialCond');
    end
   
    
    for ii = 1:length(impedance)
        %nb_data = impedance(ii).nb_id;
        experience_data_tmp(ii).user = user_name; 
        experience_data_tmp(ii).exp = impedance(ii).header;
        experience_data_tmp(ii).exp_idx = ii;
        experience_data_tmp(ii).K = impedance(ii).xi(1,:);
        experience_data_tmp(ii).B = impedance(ii).xi(2,:);
        experience_data_tmp(ii).M = impedance(ii).xi(3,:);
        experience_data_tmp(ii).r2 = impedance(ii).r2_pos;  
        experience_data_tmp(ii).ti = impedance(ii).t_pert;          
    end
    k = k - 1;
    if ~exist('impedance_data', 'var')
        impedance_data = experience_data_tmp;
    else
        impedance_data = [impedance_data,experience_data_tmp];
    end
    
    clear impedance experience_data_tmp
end

save('users_impedance.mat', 'impedance_data', '-v7.3')