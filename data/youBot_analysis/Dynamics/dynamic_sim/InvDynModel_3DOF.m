function tau = InvDynModel_3DOF(f_env,th,dth,ddth, varargin)
% Dynamic equation of the KUKA robot
% where:
%        - M,C,G are the inertia, Coriolis and gravity matrices
%        - Fv, Fs are the viscous, and static friction matrices
%        - tau is a vector of joint torques
%        - q, dq, ddq joint angles, velocities and accelerations
%        - fenv must be the force exerted on the robot by the environment,
%             that is equal to the force provided by the sensor

    options.C = 1;
    options.G = 1;
    options.Fv = 1;
    options.Fs = 1;
    
    for ii=1:2:length(varargin)
        param = varargin{ii};
        val = varargin{ii+1};
        if strcmpi(param, 'IsC')
            % is Corriolis
            options.C = val;
        elseif strcmpi(param, 'IsG')
            % is Gravity
            options.G = val;
        elseif strcmpi(param, 'IsFv')
            % is viscous friction
            options.Fv = val;
        elseif strcmpi(param, 'IsFs')
            % is static friction
            options.Fs = val;
        elseif strcmpi(param, 'nu')
            % is static friction
            nu = val;
        else
           error(['Option ''' param ''' not recognized']);
        end
    end

    th2 = th(1);
    th3 = th(2);
    th4 = th(3);
    
    dth2 = dth(1);
    dth3 = dth(2);
    dth4 = dth(3);
    
    if ~exist('nu', 'var')
       nu = 0.65; 
    end
    
    % Corriolis
    if options.C
        C = C_3DOF(dth2,dth3,dth4, th3,th4);
    else
        C = zeros(3,3);
    end
    % gravity
    if options.G
        G = G_3DOF(th2,th3,th4);
    else
        G = zeros(3,1);
    end
    % viscous friction
    if options.Fv
        Fv = diag([0.5,0.37,0.7]);
    else
        Fv = zeros(3,3);
    end
    % static friction
    if options.Fs
        Fs = Fs_3DOF(dth, f_env);
    else
        Fs = zeros(3,1);
    end
     
    tau = M_3DOF(th3,th4)*ddth + C*dth + G - ...
        nu.*J0E_3DOF(th2,th3,th4)'*f_env - Fv*dth - Fs;
    
    if(isnan(tau))
        error('tau is NaN...')
    end
    
end