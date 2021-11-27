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
        impedance(ii).arx_quick();
        impedance(ii).causalSim(dt, 'NulInitialCond');
    end
    
    experience_data(k).user = user_name;
    % init struct
    experience_data(k).exp_1.K = {};
    experience_data(k).exp_1.B = {};
    experience_data(k).exp_1.M = {};
    experience_data(k).exp_1.r2 = {};
    experience_data(k).exp_2.K = {};
    experience_data(k).exp_2.B = {};
    experience_data(k).exp_2.M = {};
    experience_data(k).exp_2.r2 = {};
    experience_data(k).l_bb.K = {};
    experience_data(k).l_bb.B = {};
    experience_data(k).l_bb.M = {};
    experience_data(k).l_bb.r2 = {};
    
    for ii = 1:length(impedance)
        if impedance(ii).header == "exp_1"
            experience_data(k).exp_1.K{end+1} = impedance(ii).xi(1,:);
            experience_data(k).exp_1.B{end+1} = impedance(ii).xi(2,:);
            experience_data(k).exp_1.M{end+1} = impedance(ii).xi(3,:);
            experience_data(k).exp_1.r2{end+1} = impedance(ii).r2_pos;
        elseif impedance(ii).header == "exp_2"
            experience_data(k).exp_2.K{end+1} = impedance(ii).xi(1,:);
            experience_data(k).exp_2.B{end+1} = impedance(ii).xi(2,:);
            experience_data(k).exp_2.M{end+1} = impedance(ii).xi(3,:);
            experience_data(k).exp_2.r2{end+1} = impedance(ii).r2_pos;
        elseif impedance(ii).header == "l_bb"
            experience_data(k).l_bb.K{end+1} = impedance(ii).xi(1,:);
            experience_data(k).l_bb.B{end+1} = impedance(ii).xi(2,:);
            experience_data(k).l_bb.M{end+1} = impedance(ii).xi(3,:);
            experience_data(k).l_bb.r2{end+1} = impedance(ii).r2_pos;
        end            
    end
    k = k - 1;
    clear impedance
end

save('users_impedance.mat', 'experience_data', '-v7.3')