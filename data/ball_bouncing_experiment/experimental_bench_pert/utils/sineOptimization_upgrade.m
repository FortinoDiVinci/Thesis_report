function [traj_opt, xmulti, errormulti] = sineOptimization_upgrade(t, sig, idx_p, varargin)
% sineOptimization
% The virtual signal, or unperturbed signal, is computed thanks to a sine
% optimization fitting
%
% Inputs:
% t: time vector of the complete signal   Nx1
% sig: completet input signal vector      Nx1
% idx_p: perturbation index     
% Varargin
% pertLength: perturbations length         default = 65 samples
% ufitLength: length of the upper bound of the signal use for the optim., 
% default = 100 samp.
% lfitLength: length of the lower bound of the signal use for the optim., 
% default = 100 samp.
% outputIndex: output indexes (1 is the perturbation timing) Mx1
% nbSine: number of different sine that will be used
% linearComp: at + b will be added to the signal
%
% Ouput:
% sig_virt: virtual signal vector   Mx1
%

% default arguments
p_l = 65;
fit_l = 70;
out_idx = idx_p + (-199:199); %499 samples centered on idx_p
nb_sine = 3;
linear_c = 0;
nb_start_pts = 1;
st_pts = [];
solver_name = 'lsqcurvefit';

if ~isempty(varargin)
    for ii = 1:2:length(varargin)
        switch(varargin{ii})
            case 'pertLength'
                p_l = varargin{ii+1};
                if mod(p_l,1) ~= 0
                    error("The perturbation length, should be provided in terms of samples."+...
                        " Therefore it should be an integer")
                end
            case 'uFitLength'
                u_fit_l = varargin{ii+1};
                if mod(fit_l,1) ~= 0
                    error("The upper fit length, should be provided in terms of samples."+...
                        " Therefore it should be an integer")
                end
            case 'lFitLength'
                l_fit_l = varargin{ii+1};
                if mod(fit_l,1) ~= 0
                    error("The lower fit length, should be provided in terms of samples."+...
                        " Therefore it should be an integer")
                end
            case 'outputIndex' 
                out_idx = varargin{ii+1};
                if (out_idx(1) < 1) || ...
                        out_idx(end) > length(t)
                    warning("The output vector goes beyond the original signal boundaries.")
                end
            case 'nbSine'
                nb_sine = varargin{ii+1};
                if mod(nb_sine,1) ~= 0 || nb_sine < 0
                    error("The number of sines must be a positive integer.")
                end
            case 'linearComp'
                linear_c = varargin{ii+1};
                % will be treated as a boolean
            case 'multiStart'
                nb_start_pts = floor(varargin{ii+1});
            case 'feedStartingPts'
                st_pts = varargin{ii+1};
            case 'solverName'
                solver_name = varargin{ii+1};
            otherwise
                error('Unknown argument')
        end
    end
end

starting_points_manualy_fed = 0;
% check fed starting points
if ~isempty(st_pts)
    % starting points have been fed, checking if the amount is right
    if linear_c
        lin_val = 2;
    else
        lin_val = 0;
    end
    if length(st_pts) ~= 3*nb_sine + lin_val
        warning("The starting points for the parameters optimisation are not"...
            +" sufficient. An array of " + string(3*nb_sine + lin_val), + ...
            " elements is required. The vector provided will be ignored.")
        % TODO add the possibility to feed only some of the starting points
    else
        starting_points_manualy_fed = 1;
    end
end

%initial parameters
if starting_points_manualy_fed
    param_init = st_pts;
else
    if linear_c
        param_init(3*nb_sine + 2) = 1e-3; % b
        param_init(3*nb_sine + 1) = 1e-3; % a
    end

    for i = 1:nb_sine
        if i == 1 % A i
            param_init(3*(i-1) + 1) = 1;
        else
            param_init(3*(i-1) + 1) = param_init(3*(i-2) + 1)/2;
        end
        param_init(3*(i-1) + 2) = 0.9*(2*i-1);                   % f i
        param_init(3*(i-1) + 3) = max(pi/2 - (i-1)*pi/2, 0);     % phi i 
    end
end

% boundaries
lb = zeros(size(param_init));
ub = zeros(size(param_init));
for i = 1:nb_sine  
    % Ai bounds
    %lb(3*(i-1) + 1) = 0; % no negative gains
    ub(3*(i-1) + 1) = 10;% force amplitude    
    % f i lower bound
%     if i ~= 1 
%         % lower bound is the previous initial frequency
%         lb(3*(i-1) + 2) = param_init(3*(i-2) + 2);
%     end
    lb(3*(i-1) + 2) = 0;
    ub(3*(i-1) + 2) = 10;
    lb(3*(i-1) + 3) = -pi/2;
    ub(3*(i-1) + 3) = pi/2;
end
% for i = 1:nb_sine
%     % f i upper bound
%     if i == nb_sine 
%         ub(3*(i-1) + 2) = 12; % max is set to 12 Hz
%     else
%         % upper bound is the next initial frequency
%         ub(3*(i-1) + 2) = param_init(3*(i) + 2);
%     end
% end
if linear_c
    lb(3*nb_sine + 2) = -100;
    ub(3*nb_sine + 2) = 100;
    lb(3*nb_sine + 1) = -100;
    ub(3*nb_sine + 1) = 100;
end

% learning set
optim_indexes = ([idx_p-l_fit_l:idx_p, idx_p+p_l:idx_p+p_l+u_fit_l]);
xd = t(optim_indexes);
yd = sig(optim_indexes);

% problem definition
if strcmp(solver_name, 'lsqcurvefit')
    problem = createOptimProblem('lsqcurvefit', 'x0', param_init, 'objective',...
    @(par,xd)sineFunc(par, xd), 'lb', lb, 'ub', ub, 'xdata', xd, 'ydata', yd);
elseif strcmp(solver_name, 'lsqnonlin')
    problem = createOptimProblem('lsqnonlin', 'x0', param_init, 'objective',...
        @(par)sineFunc(par, xd)-yd, 'lb', lb, 'ub', ub);
else
    error('Unknown solver name')
end
% multiple starting points opt
ms = MultiStart('Display','off','UseParallel',true);
[xmulti,errormulti] = run(ms, problem, nb_start_pts);
 
% optimal parameters
for i = 1:nb_sine
    param_opt.A(i) = xmulti(3*(i-1) + 1);
    param_opt.f(i) = xmulti(3*(i-1) + 2);
    param_opt.phi(i) = xmulti(3*(i-1) + 3);
end
if linear_c
    param_opt.b = xmulti(end); 
    param_opt.a = xmulti(end-1);
end

t_opt = t(out_idx);
traj_opt = sineSumSignal(param_opt, t_opt);
  
end

% sine opti function
function traj_sine = sineFunc(par,t)
    
    if mod(length(par),3) == 0
        % no linear parameters
        for i = 1:length(par)/3
            par_m.A(i) = par(3*(i-1) + 1);
            par_m.f(i) = par(3*(i-1) + 2);
            par_m.phi(i) = par(3*(i-1) + 3);
        end
    else
        %linear parameters at + b
        for i = 1:floor(length(par)/3)
            par_m.A(i) = par(3*(i-1) + 1);
            par_m.f(i) = par(3*(i-1) + 2);
            par_m.phi(i) = par(3*(i-1) + 3);
        end
        par_m.a = par(end-1);
        par_m.b = par(end);
    end
    
    traj_sine = sineSumSignal(par_m,t);   
end

% sine opti function
function cost = sineOptFunc(par,t,sig)
    
    if mod(length(par),3) == 0
        % no linear parameters
        for i = 1:length(par)/3
            par_m.A(i) = par(3*(i-1) + 1);
            par_m.f(i) = par(3*(i-1) + 2);
            par_m.phi(i) = par(3*(i-1) + 3);
        end
    else
        %linear parameters at + b
        for i = 1:floor(length(par)/3)
            par_m.A(i) = par(3*(i-1) + 1);
            par_m.f(i) = par(3*(i-1) + 2);
            par_m.phi(i) = par(3*(i-1) + 3);
        end
        par_m.a = par(end-1);
        par_m.b = par(end);
    end
    
    traj_sine = sineSumSignal(par_m,t);
    cost = sum((sig-traj_sine).^2);
    
end

% sine sum function
function traj = sineSumSignal(par,t)
    nb_sine = length(par.A);
    traj = zeros(size(t));
    for ii = 1:nb_sine
        traj = traj + par.A(ii)*sin(2*pi*par.f(ii)*t + par.phi(ii));
    end
    % linear param (optional)
    if isfield(par, 'a')
        traj = traj + par.a.*t;
    end
    if isfield(par, 'b')
        traj = traj + par.b;
    end
end