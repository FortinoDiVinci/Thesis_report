function [outputArg1,outputArg2] = impedanceStatAnalytics(delta_z, delta_f, imp_data_list)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Statistical analysis of the errors of the impedance identification
% methods
%
% Inputs:
%
% delta_z: DIFF_TRAJECT obj for the position
% delta_f: DIFF_TRAJECT obj for the force
% imp_data_list: IMPEDANCE_DATA obj list for the different methodologies

% signal pre-processing
dt = 1e-3;
[b,a] = butter(7,5/(1/(2*dt)),'low'); 
sig = filtfilt(b,a,delta_z.complete_traject);
dsig = Iu_diffcent(delta_z.time,sig);
idx_pks = crossing(dsig);
% select only upper peaks
if idx_pks(1) > idx_pks(2) 
    idx_pks = idx_pks(1:2:end); % keeps only upper peaks
else
    idx_pks = idx_pks(2:2:end); % keeps only upper peaks
end
% get rid of potentially irrelevant peaks
% TODO: validate on a large bunch of example
avg_pk_val = mean(delta_z.complete_traject(idx_pks));
std_pk_val = std(delta_z.complete_traject(idx_pks));
idx_pks(delta_z.complete_traject(idx_pks) < avg_pk_val-std_pk_val) = [];
% delete peaks that are too close, less than 600ms, to avoid duplicates
duplicates = (diff(delta_z.time(idx_pks)) < 0.6);
del_list = NaN(size(duplicates(duplicates > 0)));
% TODO: proper suppression of outliers
idx_pks(1) = [];
idx_pks(end) = [];
j = 0;
for ii = 1:length(duplicates)
    if duplicates(ii) == 0
        continue;
    end  
    j = j + 1;
    if delta_z.complete_traject(idx_pks(ii)) > delta_z.complete_traject(idx_pks(ii+1))
        del_list(j) = ii+1;
    else
        del_list(j) = ii;
    end 
    ii = ii+1; % to skip the next one
end
idx_pks_tmp = idx_pks;
idx_pks(del_list) = []; % the lower z value of the 'duplicates' is deleted

avg_freq = 1/mean(diff(delta_z.time(idx_pks)));

figure
hold on
plot(delta_z.time, delta_z.complete_traject)
plot(delta_z.time(idx_pks), delta_z.complete_traject(idx_pks), 'p')
%plot(delta_z.time, sig, '--')
%plot([1,delta_z.time(end)], [avg_pk_val - std_pk_val, avg_pk_val - std_pk_val])
title('Input signal pr-eprocessing')
legend('original signal', 'up. pks spot.')

cycles = splitCycles(delta_z.complete_traject, delta_f.complete_traject, ...
    delta_z.time, idx_pks, delta_z.pert_ind, delta_z.pert_val);

% phase1 = 1:floor(0.10*length(cycles(1).position));
% phase2 = ceil(0.10*length(cycles(1).position)):...
%     floor(0.55*length(cycles(1).position));
% phase3 = ceil(0.55*length(cycles(1).position)):...
%     floor(0.75*length(cycles(1).position));
% phase4 = ceil(0.75*length(cycles(1).position)):...
%     floor(0.95*length(cycles(1).position));
% %
% figure
% hold on
% plot(cycles(1).position)
% plot(phase1, cycles(1).position(phase1), '--')
% plot(phase2, cycles(1).position(phase2), '--')
% plot(phase3, cycles(1).position(phase3), '--')
% plot(phase4, cycles(1).position(phase4), '--')

ph1 = 0.1;
ph2 = 0.55;
ph3 = 0.75;
ph4 = 0.95;
pert_list = [];
% perturbation classification
for cyc_i = cycles   
    if ~cyc_i.isPerturbed
        continue
    end
    % perturbed cycles only
    cyc_len = length(cyc_i.position);
    cyc_pert_idx = cyc_i.pert_idx;
    cyc_pert_ph = cyc_pert_idx/cyc_len;
    % phase sorting
    if cyc_pert_ph < ph1 || cyc_pert_ph > ph4
        phase = "upper pk";
    elseif cyc_pert_ph >= ph1 && cyc_pert_ph <= ph2
        phase = "decreasing";
    elseif cyc_pert_ph > ph2 && cyc_pert_ph < ph3
        phase = "lower pk";
    else
        phase = "rising";
    end   
    new_pert.idx =  cyc_i.pert_idx_global;
    new_pert.phase = phase;
    pert_list = [pert_list, new_pert];
end
j = 0;
nb_imp_meth = length(imp_data_list);
for ii = 1:length(pert_list)
    % TODO: tolerance ?
    while delta_z.pert_ind(ii+j) ~= pert_list(ii).idx
        j = j+1;
        if(ii+j) > length(delta_z.pert_ind)
            break
        end
    end
    for k = 1:nb_imp_meth
        pert_list(ii).impedance(k).K = imp_data_list(k).xi(1,ii+j);
        pert_list(ii).impedance(k).B = imp_data_list(k).xi(2,ii+j);
        pert_list(ii).impedance(k).M = imp_data_list(k).xi(3,ii+j);
        pert_list(ii).impedance(k).r2 = imp_data_list(k).r_2(ii+j);
        pert_list(ii).impedance(k).K_rstd = imp_data_list(k).rel_std(1,ii+j);
        pert_list(ii).impedance(k).B_rstd = imp_data_list(k).rel_std(1,ii+j);
        pert_list(ii).impedance(k).M_rstd = imp_data_list(k).rel_std(1,ii+j);
    end
end

% display
up_p = pert_list([pert_list.phase]=="upper pk");
de_p = pert_list([pert_list.phase]=="decreasing");
lo_p = pert_list([pert_list.phase]=="lower pk");
ri_p = pert_list([pert_list.phase]=="rising");
figure('DefaultAxesFontSize',14)
hold on
plot(delta_z.time, delta_z.complete_traject)
plot(delta_z.time([up_p.idx]), delta_z.complete_traject([up_p.idx]), 'p')
plot(delta_z.time([de_p.idx]), delta_z.complete_traject([de_p.idx]), 'p')
plot(delta_z.time([lo_p.idx]), delta_z.complete_traject([lo_p.idx]), 'p')
plot(delta_z.time([ri_p.idx]), delta_z.complete_traject([ri_p.idx]), 'p')
legend('z', 'pu', 'pd', 'pl', 'pr')

up_p_imp = [up_p.impedance];
de_p_imp = [de_p.impedance];
lo_p_imp = [lo_p.impedance];
ri_p_imp = [ri_p.impedance];
al_p_imp = [pert_list.impedance];
%
figure('DefaultAxesFontSize',14)
subplot(2,2,1)
hold on
for ii = 1:nb_imp_meth
    histogram([up_p_imp(ii:nb_imp_meth:end).K],5,'DisplayName',"Mth."+...
        string(ii))
end
hold off
legend show
title("Upper peak Stiffness, mean: " + sprintf('%.4f',...
    mean([up_p_imp(ii:nb_imp_meth:end).K])))
subplot(2,2,2)
hold on
for ii = 1:nb_imp_meth
    histogram([de_p_imp(ii:nb_imp_meth:end).K],5,'DisplayName',"Mth."+...
        string(ii))
end
hold off
legend show
title("Decreasing Stiffness,  mean: " + sprintf('%.4f',...
    mean([de_p_imp(ii:nb_imp_meth:end).K])))
subplot(2,2,3)
hold on
for ii = 1:nb_imp_meth
    histogram([lo_p_imp(ii:nb_imp_meth:end).K],5,'DisplayName',"Mth."+...
        string(ii))
end
hold off
legend show
title("Lower peak Stiffness,  mean: " + sprintf('%.4f',...
    mean([lo_p_imp(ii:nb_imp_meth:end).K])))
subplot(2,2,4)
hold on
for ii = 1:nb_imp_meth
    histogram([ri_p_imp(ii:nb_imp_meth:end).K],5,'DisplayName',"Mth."+...
        string(ii))
end
hold off
legend show
title("Rising Stiffness,  mean: " + sprintf('%.4f',...
    mean([ri_p_imp(ii:nb_imp_meth:end).K])))

% R2 scores
figure('DefaultAxesFontSize',14)
subplot(2,2,1)
hold on
for ii = 1:nb_imp_meth
    plot([up_p_imp(ii:nb_imp_meth:end).r2],'DisplayName',...
        "Mth."+string(ii))
    mn = mean([up_p_imp(ii:nb_imp_meth:end).r2]);
    plot([1 length(up_p_imp)/nb_imp_meth], [mn, mn],'DisplayName',...
        "Mth."+string(ii) + " mean")
end
ylabel('R2 score')
title("Upper peaks")
legend show  
subplot(2,2,2)
hold on
for ii = 1:nb_imp_meth
    plot([lo_p_imp(ii:nb_imp_meth:end).r2],'DisplayName',...
        "Mth."+string(ii))
    mn = mean([lo_p_imp(ii:nb_imp_meth:end).r2]);
    plot([1 length(lo_p_imp)/nb_imp_meth], [mn, mn],'DisplayName',...
        "Mth."+string(ii) + " mean")
end
title("Lower peaks")
legend show  
subplot(2,2,3)
hold on
for ii = 1:nb_imp_meth
    plot([de_p_imp(ii:nb_imp_meth:end).r2],'DisplayName',...
        "Mth."+string(ii))
    mn = mean([de_p_imp(ii:nb_imp_meth:end).r2]);
    plot([1 length(de_p_imp)/nb_imp_meth], [mn, mn],'DisplayName',...
        "Mth."+string(ii) + " mean")
end
ylabel('R2 score')
xlabel('Nb identifications')
title("Decreasing phase ")
legend show  
subplot(2,2,4)
hold on
for ii = 1:nb_imp_meth
    plot([ri_p_imp(ii:nb_imp_meth:end).r2],'DisplayName',...
        "Mth."+string(ii))
    mn = mean([ri_p_imp(ii:nb_imp_meth:end).r2]);
    plot([1 length(ri_p_imp)/nb_imp_meth], [mn, mn],'DisplayName',...
        "Mth."+string(ii) + " mean")
end
title("Rising phase")
legend show  
xlabel('Nb identifications')

%
figure('DefaultAxesFontSize',14)
subplot(3,1,1)
hold on
for ii = 1:nb_imp_meth
     plot([al_p_imp(ii:nb_imp_meth:end).r2], [al_p_imp(ii:nb_imp_meth:end).K],...
         'o','DisplayName', "Mth."+string(ii))
end
legend show
xlabel('R^2 score')
ylabel('N/m')
title('Stiffness')
subplot(3,1,2)
hold on
for ii = 1:nb_imp_meth
     plot([al_p_imp(ii:nb_imp_meth:end).r2], [al_p_imp(ii:nb_imp_meth:end).B],...
         'o','DisplayName', "Mth."+string(ii))
end
legend show
xlabel('R^2 score')
ylabel('N.s/m')
title('Damping')
subplot(3,1,3)
hold on
for ii = 1:nb_imp_meth
     plot([al_p_imp(ii:nb_imp_meth:end).r2], [al_p_imp(ii:nb_imp_meth:end).M],...
         'o','DisplayName', "Mth."+string(ii))
end
legend show
xlabel('R^2 score')
ylabel('kg')
title('Mass')



end

%%%

function [yd] = Iu_diffcent(y,t)

ny=length(y);

dtemps = [(t(2)-t(1));(t(3:ny)-t(1:ny-2))/2;(t(ny)-t(ny-1))];
yd=[(y(2)-y(1));(y(3:ny)-y(1:ny- 2))/2;(y(ny)-y(ny-1))]./dtemps;
end

function [ind,t0,s0,t0close,s0close] = crossing(S,t,level,imeth)
% CROSSING find the crossings of a given level of a signal
%   ind = CROSSING(S) returns an index vector ind, the signal
%   S crosses zero at ind or at between ind and ind+1
%   [ind,t0] = CROSSING(S,t) additionally returns a time
%   vector t0 of the zero crossings of the signal S. The crossing
%   times are linearly interpolated between the given times t
%   [ind,t0] = CROSSING(S,t,level) returns the crossings of the
%   given level instead of the zero crossings
%   ind = CROSSING(S,[],level) as above but without time interpolation
%   [ind,t0] = CROSSING(S,t,level,par) allows additional parameters
%   par = {'none'|'linear'}.
%	With interpolation turned off (par = 'none') this function always
%	returns the value left of the zero (the data point thats nearest
%   to the zero AND smaller than the zero crossing).
%
%	[ind,t0,s0] = ... also returns the data vector corresponding to 
%	the t0 values.
%
%	[ind,t0,s0,t0close,s0close] additionally returns the data points
%	closest to a zero crossing in the arrays t0close and s0close.
%
%	This version has been revised incorporating the good and valuable
%	bugfixes given by users on Matlabcentral. Special thanks to
%	Howard Fishman, Christian Rothleitner, Jonathan Kellogg, and
%	Zach Lewis for their input. 

% Steffen Brueckner, 2002-09-25
% Steffen Brueckner, 2007-08-27		revised version

% Copyright (c) Steffen Brueckner, 2002-2007
% brueckner@sbrs.net

% check the number of input arguments
error(nargchk(1,4,nargin));

% check the time vector input for consistency
if nargin < 2 || isempty(t)
	% if no time vector is given, use the index vector as time
    t = 1:length(S);
elseif length(t) ~= length(S)
	% if S and t are not of the same length, throw an error
    error('t and S must be of identical length!');    
end

% check the level input
if nargin < 3
	% set standard value 0, if level is not given
    level = 0;
end

% check interpolation method input
if nargin < 4
    imeth = 'linear';
end

% make row vectors
t = t(:)';
S = S(:)';

% always search for zeros. So if we want the crossing of 
% any other threshold value "level", we subtract it from
% the values and search for zeros.
S   = S - level;

% first look for exact zeros
ind0 = find( S == 0 ); 

% then look for zero crossings between data points
S1 = S(1:end-1) .* S(2:end);
ind1 = find( S1 < 0 );

% bring exact zeros and "in-between" zeros together 
ind = sort([ind0 ind1]);

% and pick the associated time values
t0 = t(ind); 
s0 = S(ind);

if strcmp(imeth,'linear')
    % linear interpolation of crossing
    for ii=1:length(t0)
        if abs(S(ind(ii))) > eps(S(ind(ii)))
            % interpolate only when data point is not already zero
            NUM = (t(ind(ii)+1) - t(ind(ii)));
            DEN = (S(ind(ii)+1) - S(ind(ii)));
            DELTA =  NUM / DEN;
            t0(ii) = t0(ii) - S(ind(ii)) * DELTA;
            % I'm a bad person, so I simply set the value to zero
            % instead of calculating the perfect number ;)
            s0(ii) = 0;
        end
    end
end

end
