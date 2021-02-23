function [traj_opt, param_opt, exit_flag] = sineOptimization(t, sig, idx_p, p_l, fit_l, out_idx)
% sineOptimization
% The virtual signal, or unperturbed signal, is computed thanks to a sine
% optimization fitting
%
% Inputs:
% t: time vector                    Nx1
% sig: input signal vector          Nx1
% idx_p: perturbation index      	
% p_l: perturbations length         default = 65 samples
% fit_l: length of the signal use for the optimization, default = 100 samp.
% out_l: output indexes             Mx1
%
% Ouput:
% sig_virt: virtual signal vector   Mx1
%

% default arguments
if nargin < 4
    p_l = 65;
end
if nargin < 5
    fit_l = 100;
end
if nargin < 6
    out_idx = idx_p + (-199:199); %499 samples centered on idx_p
end

%initial parameters
param_init(1) = 1;    % A(1)
param_init(2) = 0.9;  % f(1)
param_init(3) = pi/2; % phi(1)
param_init(4) = 0.1;  % A(2)
param_init(5) = 2.5;  % f(2)
param_init(6) = 0;    % phi(2)
param_init(7) = 0.1;  % A(3)
param_init(8) = 5.5;  % f(3)
param_init(9) = 0;    % phi(3)
param_init(10) = 0.1;  % A(4)
param_init(11) = 6.5;  % f(4)
param_init(12) = 0;    % phi(4)
param_init(13) = 1e-3; % a
param_init(14) = 1e-3; % b
% options
opt = optimset('fminunc');
opt = optimset(opt,'MaxFunEvals',2000,'Display', 'off');
% indexes that will be masked    
idx_p_i = idx_p + (0:p_l-1);
% coordinates for the optimization
idx_fit_i = [idx_p_i(1)+(-ceil(fit_l/2):1:-1), idx_p_i(end)+(1:1:fit_l)];
t_idx = t(idx_fit_i);
sig_idx = sig(idx_fit_i);
[p_argmin, ~, exit_flag, info] = fminunc(@(par)sineOptFunc(par, t_idx, ...
    sig_idx), param_init, opt);   
% optimal parameters
param_opt.A(1) = p_argmin(1);
param_opt.f(1) = p_argmin(2);
param_opt.phi(1) = p_argmin(3);
param_opt.A(2) = p_argmin(1);
param_opt.f(2) = p_argmin(5);
param_opt.phi(2) = p_argmin(6);
param_opt.A(3) = p_argmin(7);
param_opt.f(3) = p_argmin(8);
param_opt.phi(3) = p_argmin(9);
param_opt.A(4) = p_argmin(10);
param_opt.f(4) = p_argmin(11);
param_opt.phi(4) = p_argmin(12);
param_opt.a = p_argmin(13);
param_opt.b = p_argmin(14);

t_opt = t(out_idx);
traj_opt = sineSumSignal(param_opt, t_opt);
  
end

% sine opti function
function cost = sineOptFunc(par,t,sig)
    
    par_m.A(1) = par(1);
    par_m.f(1) = par(2);
    par_m.phi(1) = par(3);
    par_m.A(2) = par(4);
    par_m.f(2) = par(5);
    par_m.phi(2) = par(6);
    par_m.A(3) = par(7);
    par_m.f(3) = par(8);
    par_m.phi(3) = par(9);
    par_m.A(4) = par(10);
    par_m.f(4) = par(11);
    par_m.phi(4) = par(12);
    par_m.a = par(13);
    par_m.b = par(14);
    
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