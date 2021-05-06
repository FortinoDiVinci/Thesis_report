function tau = InvDynModel_3DOF(f_env,th,dth,ddth, varargin)
% Dynamic equation of the KUKA robot
% where:
%        - M,C,G are the inertia, Coriolis and gravity matrices
%        - Fv, Fs are the viscous, and static friction matrices
%        - tau is a vector of joint torques
%        - q, dq, ddq joint angles, velocities and accelerations
%        - fenv must be the force exerted on the robot by the environment,
%             that is equal to force provided by the sensor in the robot
%             frame

    options.C = 1;
    options.G = 1;
    options.Fv = 1;
    options.Fs = 1;
    options.offset = 1;
    
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
        elseif strcmpi(param, 'IsOffset')
            % is static friction
            options.offset = val;
        elseif strcmpi(param, 'nu')
            % is static friction
            nu = val;
        elseif strcmpi(param, 'offset')
            % is static friction
            tau0 = val;
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
       %nu = diag([0.9488, 0.9216, 0.8345]); 
       nu = diag([0.8554, 0.5780, 0.9997]); 
    else
        if all(size(ones(3)) == size(nu)) % nu provided as matrix
            if ~isdiag(nu)
                error('The nu matrix provided should be diagonal.')
            end
        elseif any(size(ones(3)) == size(nu)) % nu provided as vector
            nu = diag(nu);
        elseif all(size(1) == size(nu)) % nu provided as scalar
            nu = nu.*eye(3);
        else
            error('nu val has unexpetected dimensions.')
        end
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
    % torque offset
    if ~exist('tau0', 'var') % if tau0 was manually fed, skip this part
        if options.offset
            %tau0 = [1.0234;1.0516;1.0901];
            tau0 = [1.0050; 0.6119; 0.8268];
        else
            tau0 = zeros(3,1);
        end
    else
        if any(size(tau0) ~= size(zeros(3,1)))
            error('Offset size should be (3,1)')
        end
    end
     
    tau = M_3DOF(th3,th4)*ddth + C*dth + G + ...
        nu*J0E_3DOF(th2,th3,th4)'*f_env - Fv*dth - Fs + tau0;
    
    if(isnan(tau))
        error('tau is NaN...')
    end
    
end