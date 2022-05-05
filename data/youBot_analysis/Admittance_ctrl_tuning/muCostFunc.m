function [cost] = muCostFunc(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    fc_w1 = x(1);
    fc_w2 = x(2);
    g_w1 = x(3);
    g_w2 = x(4);

    load('../Dynamics/youbot_lin_model_z.mat');
    Sigma = youbot_lin_model_ss_z;
    clear youbot_lin_model_ss_z

    Sigma = minreal(Sigma, [], false); % delete the 3 ev states

    % equilibrium (around which the model was linearized)
    q0 = [1.676; -4.363; 1.497];
    dq0 = [0;0;0];
    ev0 = [0;0;0];

    % env
    K = ureal('K',160,'Percentage',50);
    B = 11;
    M = 0.42;
    tau_m = 1/(10*2*pi);

    %% PI analysis (for comparison)
    s = tf('s');
    % PI ctrl
    Kp = 0.015;
    Ki = 0.08;
    H_pi = Ki/s + Kp;
    
    load_system('analysis_PI_ctrl_lin_uncertain_model_z');
    io(1) = linio('analysis_PI_ctrl_lin_uncertain_model_z/fin',1, 'input');
    io(2) = linio('analysis_PI_ctrl_lin_uncertain_model_z/b',1, 'input');
    io(3) = linio('analysis_PI_ctrl_lin_uncertain_model_z/w',1, 'input');
    io(4) = linio('analysis_PI_ctrl_lin_uncertain_model_z/Sum',1, 'output'); % eps
    io(5) = linio('analysis_PI_ctrl_lin_uncertain_model_z/Admittance PI',1, 'output'); % u
    io(6) = linio('analysis_PI_ctrl_lin_uncertain_model_z/Sum1',1, 'output'); % w
    io(7) = linio('analysis_PI_ctrl_lin_uncertain_model_z/endpoint model of youBot',1, 'output'); % s
    argout_PI = ulinearize('analysis_PI_ctrl_lin_uncertain_model_z',io);
    % input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
    % output 1) is eps, 2) is u (cmd), 3) is r (fz)

    S = ss(argout_PI.a, argout_PI.b(:,1), argout_PI.c(1,:), argout_PI.d(1,1)); % sensivity
    T = ss(argout_PI.a, argout_PI.b(:,1), argout_PI.c(3,:), argout_PI.d(3,1)); % comp. sens.
    %CL = ss(argout_PI.a, argout_PI.b(:,1), argout_PI.c(4,:), argout_PI.d(4,1)); % close loop.
    
    w = linspace(0.8*2*pi,5*2*pi,50);
    %[mag_s_pi,~,~] = bode(S,w);
    mag_s_pi = bode(T,w);
    
    %% Weights (to be tunned)
    W1 = 1/makeweight(1e-6,[2*pi*fc_w1,1],g_w1,0,2);
    W2 = 1/makeweight(g_w2,[2*pi*fc_w2,1],0.1);
    
    %% Hinf synthesis
    load_system('synth_hinf_ctrl_lin_uncertain_model_z')
    io_hinf(1) = linio('synth_hinf_ctrl_lin_uncertain_model_z/fin',1, 'input');
    io_hinf(2) = linio('synth_hinf_ctrl_lin_uncertain_model_z/cmd',1, 'input');
    io_hinf(3) = linio('synth_hinf_ctrl_lin_uncertain_model_z/W1',1, 'output');
    io_hinf(4) = linio('synth_hinf_ctrl_lin_uncertain_model_z/W2',1, 'output');
    io_hinf(5) = linio('synth_hinf_ctrl_lin_uncertain_model_z/Sum',1, 'output');

    H = ulinearize('synth_hinf_ctrl_lin_uncertain_model_z',io_hinf);
    nb_meas = 1;  % nb input for controller
    nb_cmd = 1;   % nb cmd
    %    
    opts = musynOptions('Display','off');
    [Hinf_ctrl,~,info] = musyn(H,nb_meas,nb_cmd,opts);
    gamma = info(end).gamma;
    
    H_inf_tf = zpk(Hinf_ctrl);
    
    % reducing the degree of the controller 
%     z_sel = H_inf_tf.Z{1}((H_inf_tf.Z{1} > -1e3) & (H_inf_tf.Z{1} < -5e-1));
%     p_sel = H_inf_tf.P{1}((H_inf_tf.P{1} > -1e3) & (H_inf_tf.P{1} < -5e-1));
%     nb_integrator = sum(~(H_inf_tf.P{1} < -5e-1));
%     p_sel = [p_sel; zeros(nb_integrator,1)];
%     Hinf_ctrl_red_tmp = zpk(z_sel,p_sel,H_inf_tf.K);
%     Hinf_ctrl_red = ss(minreal(Hinf_ctrl_red_tmp, 0.15));
    
%     [Hinf_ctrl_bal,G_Hinf_ctrl]=balreal(Hinf_ctrl);
%     elim = G_Hinf_ctrl< 0.001;
%     Hinf_ctrl_red = modred(Hinf_ctrl_bal,elim);
    
%     tmp = Hinf_ctrl;
%     Hinf_ctrl = Hinf_ctrl_red;
    
    clear io
    load_system('analysis_hinf_ctrl_lin_uncertain_model_z')
    io(1) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/fin',1, 'input');
    io(2) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/b',1, 'input');
    io(3) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/w',1, 'input');
    io(4) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/Sum',1, 'output'); % eps
    io(5) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/H_inf_ctrl',1, 'output'); % u
    io(6) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/Sum1',1, 'output'); % w
    io(7) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/endpoint model of youBot',1, 'output'); % s
    argout_hinf = ulinearize('analysis_hinf_ctrl_lin_uncertain_model_z',io);
    % input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
    % output 1) is eps, 2) is u (cmd), 3) is r (fz)

%     Hinf_ctrl = tmp;
    
    S2 = ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1)); % sensivity
    T2 = ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(3,:), argout_hinf.d(3,1)); % comp. sens.
    CL2 = ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(4,1)); % comp. sens.
    
    if isstable(CL2) && gamma < 1
    
        [mag_s_hinf,~,~] = bode(T2,w);
    
        cost_bode = 10*sum((squeeze(mag_s_hinf)-squeeze(mag_s_pi)).^2)/50; % sum of quadratic diff
        %cost_gamma = gamma*abs(gamma - 1);
        %cost_gamma = 10*(gamma - 0.9)^2;
        cost_S_sig = log(max(sigma(S2)));%/2;
        cost_T_sig = log(max(sigma(T2)));%/3;
        
        t = (0:1e-3:10);
        u = sin(2*pi*0.9.*t);
        y_hinf_ga = lsim(CL2, u, t);
        
        cost_transp_mag = exp(-100*max(y_hinf_ga));
%         stepResp = stepinfo(H_clga);
%         stepResp.SettlingTime        
        [~,idx_y] = max(y_hinf_ga(4000:5000));
        [~,idx_u] = max(u(4000:5000));
        
        cost_phase_delay = abs((idx_y - idx_u)/1e3);

        %cost = cost_bode + cost_gamma + cost_S_sig + cost_T_sig + cost_transp_mag;%exp(-1/cost_bode) + exp(-1/cost_gamma) + exp(-1/cost_S_sig) + exp(-1/cost_T_sig);    
        %cost = cost_bode + cost_gamma + cost_transp_mag;%exp(-1/cost_bode) + exp(-1/cost_gamma) + exp(-1/cost_S_sig) + exp(-1/cost_T_sig); 
        cost = cost_bode + cost_phase_delay + cost_transp_mag + cost_S_sig + cost_T_sig;
        
    else
        cost_gamma = (gamma - 1)^2;
        cost = 10 + cost_gamma;
    end
        
end