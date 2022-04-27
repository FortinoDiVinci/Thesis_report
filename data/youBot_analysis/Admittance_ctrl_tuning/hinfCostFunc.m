function [cost] = hinfCostFunc(x)
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
    K = 150;
    B = 10;
    M = 0.45;
    tau_m = 1/(10*2*pi);

    %% PI analysis (for comparison)
    s = tf('s');
    % PI ctrl
    Kp = 0.015;
    Ki = 0.08;
    H_pi = Ki/s + Kp;
    
    argout_PI = linmod('analysis_PI_ctrl_lin_model_z');
    % input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
    % output 1) is eps, 2) is u (cmd), 3) is r (fz)

    S = ss(argout_PI.a, argout_PI.b(:,1), argout_PI.c(1,:), argout_PI.d(1,1)); % sensivity
    T = ss(argout_PI.a, argout_PI.b(:,1), argout_PI.c(3,:), argout_PI.d(3,1)); % comp. sens.

    w = linspace(0.8*2*pi,5*2*pi,50);
    [mag_s_pi,~,~] = bode(S,w);
    
    %% Weights (to be tunned)
    W1 = 1/makeweight(1e-6,[2*pi*fc_w1,1],g_w1,0,2);
    W2 = 1/makeweight(g_w2,[2*pi*fc_w2,1],0.1);
    
    %% Hinf synthesis
    [A_p,B_p,C_p,D_p] = gettf('synth_hinf_ctrl_lin_model_z',1:3,1:2);
    nb_meas = 1;  % nb input for controller
    nb_cmd = 1;   % nb cmd
    %
    H = ss(A_p,B_p,C_p,D_p);
    
    [Hinf_ctrl,bf,gamma] = hinfsyn(H,nb_meas,nb_cmd,'display','off');
    
    H_inf_tf = zpk(Hinf_ctrl);
    
    % reducing the degree of the controller 
%     z_sel = H_inf_tf.Z{1}((H_inf_tf.Z{1} > -1e3) & (H_inf_tf.Z{1} < -5e-1));
%     p_sel = H_inf_tf.P{1}((H_inf_tf.P{1} > -1e3) & (H_inf_tf.P{1} < -5e-1));
%     nb_integrator = sum(~(H_inf_tf.P{1} < -5e-1));
%     p_sel = [p_sel; zeros(nb_integrator,1)];
%     Hinf_ctrl_red_tmp = zpk(z_sel,p_sel,H_inf_tf.K);
%     Hinf_ctrl_red = ss(minreal(Hinf_ctrl_red_tmp, 0.15));
    
    [Hinf_ctrl_bal,G_Hinf_ctrl]=balreal(Hinf_ctrl);
    elim = G_Hinf_ctrl< 0.001;
    Hinf_ctrl_red = modred(Hinf_ctrl_bal,elim);
    
    tmp = Hinf_ctrl;
    Hinf_ctrl = Hinf_ctrl_red;
    
    argout_hinf = linmod('analysis_hinf_ctrl_lin_model_z');
    % input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
    % output 1) is eps, 2) is u (cmd), 3) is r (fz)

    Hinf_ctrl = tmp;
    
    S2 = ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1)); % sensivity
    T2 = ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(3,:), argout_hinf.d(3,1)); % comp. sens.
    CL2 = ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(4,1)); % comp. sens.
    
    if isstable(CL2)
    
        [mag_s_hinf,~,~] = bode(S2,w);
    
        cost_bode = 10*sum((squeeze(mag_s_hinf)-squeeze(mag_s_pi)).^2)/50; % sum of quadratic diff
        %cost_gamma = gamma*abs(gamma - 1);
        cost_gamma = 10*(gamma - 0.95)^2;
        cost_S_sig = log(max(sigma(S2)))/2;
        cost_T_sig = log(max(sigma(T2)))/3;
        

        cost = cost_bode + cost_gamma + cost_S_sig + cost_T_sig;%exp(-1/cost_bode) + exp(-1/cost_gamma) + exp(-1/cost_S_sig) + exp(-1/cost_T_sig);    
    
    else
        cost = 10;
    end
        
end

