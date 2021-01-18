function f_virt = computeVirtualForce(t,f,idx_p,p_l,df)
% computeVirtualForce
% The virtual force, or unperturbed force, is computed thanks to successive
% filtering and by deleting the continuous component of the perturbations
%
% Inputs:
% t: time vector                    Nx1
% f: force vector                   Nx1
% idx_p: perturbations indexes      Mx1
% p_d: perturbations length         default = 65 samples
% df: designfilt                    default (see below)
%
% Ouput:
% f_virt: virtual force vector      Nx1
%

dt = mean(diff(t));
% default arguments
if nargin < 4
    p_l = 65;
end
if nargin < 5
    % default low pass filter
    Fs = 1/dt; Fp = 8; Fc = 8.5; At = 20;
    df = designfilt('lowpassfir','PassbandFrequency',Fp,...
  'StopbandFrequency',Fc,'PassbandRipple',0.1,...
  'StopbandAttenuation',At,'SampleRate',Fs);
end

f_order = filtord(df);
f_filt = filtfilt(df, f);
f_wo_p = f_filt;
%f_virt = f_filt;
nb_pert = length(idx_p);
nb_rep = floor(3*f_order/p_l) + 10; % for stability of the filter response

% main algo iteration over all the perturbations
% the perturbations need to enough spaced so that their respective effects
% do not overlap, otherwise, this method might fail
for i = 1:nb_pert
    
    idx_p_i = idx_p(i) + (0:p_l-1);  
    t_p = t(idx_p_i);
    f_p = f(idx_p_i);
    % extraction of the perturbation to separate it from the rest of the
    % signal, and collect a single perturbation filtered
    f_p_rep = repmat(f_p,1,nb_rep);
    p_rep_filt = filtfilt(df, f_p_rep);
    stable_idx = (floor(1.5*f_order/p_l) + 5)*p_l;
    p_filt = p_rep_filt(stable_idx:stable_idx + p_l - 1);
    % line between the begining of the perturbation and a point 65ms after
    interp_line = interp1([t_p(1),t_p(end)],[f_p(1),f_p(end)], t_p)';
    % the continuous part is then substracted to the filtered signal
    contin_comp = p_filt - interp_line;
    f_wo_p(idx_p_i) = f_wo_p(idx_p_i) - contin_comp;
end
% Then the filtered signal with the perturbation's continuous component 
% substracted is filtered to correct all discontinuities
f_virt = filtfilt(df, f_wo_p);

end

