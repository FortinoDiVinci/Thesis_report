function [cost] = bouncing_ball_tee_model(t_d, t_s_input, lbd, h_0, h_2)
    
    %% Variables %%

    % Initial conditions --------------------------------------------------
    Vb0 = 0;                        % Ball speed (m/s)
    Zb0 = 0.55;                     % Ball height (m)
    Hp = 0.55;                      % Target height (m)
    alpha = 0.48;                   % Restitution coeff paddle/ball
    g = 9.81;                       % Gravity acc (SI)
    t_max = 30;                     % Trial duration (s)
    t_init = 1.72;                  % Init duration of the oscillator (s)
    t_new_h = 20;                   % Time of new target height
    t_s_in = t_s_input;             % CPG input entrainment sampling period (s)
    % for now t_s_in should be a multiple of t_s
    t_s = 0.003;                    % Sampling time (s)
    Ar0 = 14;                       % Excitability c
    Pt0 = 0.66;                     % Eigen period of the oscillator (s)

    % CPG -----------------------------------------------------------------
    delay = uint32(t_d/(t_s*1e3));  % Delay upon the perception of the ball
    lambda = lbd;                   % Adaptation gain of cpg input
    h0 = h_0;                       % Sensor input gain
    y_out = 0;                      % Output of the oscillator
    x1_out = 0;                     % Init state of the oscillator
    x2_out = 1;                     % Init state of the oscillator
    f1_out = 0;                     % Init state of the oscillator
    f2_out = 1;                     % Init state of the oscillator

    % Impedance of the arm ------------------------------------------------
    I = 0.1;                        % Inertia
    B = 1.8;                        % Damping
    K = 25;                         % Stiffness
    h2 = h_2;                       % Gain of the equilibrium position input
    arm_pos_0 = 0.0;                % Arm position along z axis
    arm_speed_0 = 0.0;              % Arm speed along z axis
    arm_acc_0 = 0.0;                % Arm acceleration along z axis

    % Time vectors --------------------------------------------------------
    t0 = (0:t_s:t_init);
    t_1 = (t_init:t_s:t_new_h);
    t_exp = (t_init:t_s:t_max);
    t_tot = [t0 , t_exp];

    % Stored data ---------------------------------------------------------
    n_step = uint32((t_max - t_init) / t_s);
    Zb_d = zeros(n_step,1);                 % ball position along z
    Za_d = zeros(n_step,1);                 % arm position along z
    Vb_d = zeros(n_step,1);                 % ball speed along z 
    Va_d = zeros(n_step,1);                 % arm speed along z
    
    %% Initialization %%
    
    cpg = CPG(Ar0, Pt0, x1_out, x2_out, f1_out, f2_out, t_s, 0);
    arm = ARM(K, B, I, arm_pos_0, arm_speed_0, arm_acc_0, t_s, 0);
    
    for ii=1:length(t0)
        % CPG ordinary diff equation
        matsuoka_output(cpg); 
        % ARM ordinary diff equation
        tee_arm_model(arm, h2*cpg.y, h2*cpg.dy);  
        Za_d(ii) = arm.pos;
        Va_d(ii) = arm.speed;
    end
    
    %% Begining of the experiment %%
    
    % Initialization ---------------------------------------------------------
    cpg.t = 0;   
    arm.t = 0;
    cpg.input = 0;
    t_i = 1;                                    % iteration since last impact
    Nb_impact = 0;
    Pa = 1/9.38;
    lst_err = 0;
    lst_err_cont = 0;
    nb_apex = 0;                    % to avoid good scoring for no bouncing

    for ii=2:length(t_exp)
        % -------------------- Continuous loop ----------------------------- %
        if Nb_impact >= 1                      % starts at 2nd impact
            if mod(ii, t_s_in/t_s) == 0        % change input every xx ms
                if ii <= delay
                    cpg.input = 0;
                else
                    cpg.input = h0*Vb_d(ii-delay); % entrainment of the CPG
                end
            end
        end
        matsuoka_output(cpg); 

        if ii == length(t_1)
            Hp = 0.75;
        end

        % --------------------- Disturbance -------------------------------- %
           % not implemented in this version %
       
        tee_arm_model(arm, h2*cpg.y, h2*cpg.dy);     

        Za_d(ii) = arm.pos; 
        Va_d(ii) = arm.speed;
        
        lst_err_cont = lst_err_cont + 1e-3*(Za_d(ii) - Hp)^2;
        % balistic equations
        Zb_d(ii+1) = -0.5*g*t_tot(t_i+1)^2 + Vb0*t_tot(t_i+1) + Zb0; % ball position
        Vb_d(ii) = (Zb_d(ii+1) - Zb_d(ii))/t_s;         % ball speed

        % -------------------- Impact detection ---------------------------- %
        if Zb_d(ii+1) <= Za_d(ii)  
            t_i = 1;                            % reset impact time
            Nb_impact = Nb_impact + 1;
            Zb0 = Zb_d(ii);                     % ball pos af. impact
            Vb0 = -alpha*(Vb_d(ii) - Va_d(ii)) + Va_d(ii); % ball speed at impact
            Zb_d(ii+1) = -0.5*g*t_tot(t_i+1)^2 + Vb0*t_tot(t_i+1) + Zb0; % theorical ball position
            Zb_d(ii) = max(Zb_d(ii+1), arm.pos);          % real ball position
            Vb_d(ii) = (Zb_d(ii+1) - Zb_d(ii))/t_s;     % ball speed

            Pa = 2*Vb0/g;                       % ball period estimation
            Ha = (Vb0^2)/(2*g) + Zb0;           % ball apex estimation
        end

        % -------------------- CPG param correction ------------------------ %
        if Nb_impact > 0 && Vb_d(ii) <= 0 && Vb_d(ii - 1) > 0
            err = Ha - Hp;
            nb_apex = nb_apex + 1;
            lst_err = lst_err + err^2;
            if err ~= 0
                cpg.Ar = max(0, cpg.Ar + lambda*err);
                cpg.Pt = Pa;
            end 
        end
        t_i = t_i + 1;
    end
    
    cost = lst_err;
    
    % too few apexes
    if nb_apex == 0
        cost = 1e10;
    elseif nb_apex <= 20
        cost = cost + 1e9*1/nb_apex;
    end
    
end

