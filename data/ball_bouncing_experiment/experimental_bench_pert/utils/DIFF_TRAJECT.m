classdef DIFF_TRAJECT < handle

    
properties

    estim_window;  % number of sample for the desired trajectories
    interp_window; % number of samples for the interpolation 
    nb_traject;    % number of trajectories to be estimated

    pert_ind;      % indexes at which trajectories will be estimated
    pert_val;      % value of the perturbation for sorting purpose
    delay;         % delay from the perturbation index (in samples)

    complete_traject;
    time;

    t_traject;
    traject;
    virt_traject;
    tmp_diff_traject;

    diff_traject;        
    d_diff_traject;
    dd_diff_traject;

end

methods

    function self = DIFF_TRAJECT(estimation_window_size, interpolation_window_size, ...
            complete_trajectory, complete_time_frame, perturbation_indexes, ...
            perturbation_values, delay_sample)

        switch (nargin)
            case 0
                % default constructor
                self.complete_traject = [];
                self.time = [];
                self.pert_ind = [];
                self.pert_val = [];
                self.delay = [];
                
                self.t_traject = [];
                self.traject = [];            
                self.virt_traject = [];  
                self.tmp_diff_traject = [];  
                self.diff_traject = [];  
                self.d_diff_traject = [];  
                self.dd_diff_traject = [];  
                
            case 7
                self.interp_window = interpolation_window_size;
                self.estim_window = estimation_window_size;
                self.nb_traject = length(perturbation_indexes);

                self.complete_traject = complete_trajectory;
                self.time = complete_time_frame;
                self.pert_ind = perturbation_indexes;
                self.pert_val = perturbation_values;
                self.delay = delay_sample;

                self.t_traject = NaN(self.estim_window + 4, self.nb_traject);
                self.traject = NaN(self.estim_window + 4, self.nb_traject);            
                self.virt_traject = NaN(self.estim_window + 4, self.nb_traject);
                self.tmp_diff_traject = NaN(self.estim_window + 4, self.nb_traject);

                self.diff_traject = NaN(self.estim_window, self.nb_traject); 
                self.d_diff_traject = NaN(self.estim_window, self.nb_traject); 
                self.dd_diff_traject = NaN(self.estim_window, self.nb_traject); 
            otherwise
                error('Wrong number of arguments')
        end
    end

    function self = computeDiffTraject(self, varargin)

        differential_direction = 1; % x0 - x

        if ~isempty(varargin)
            for ii = 1:2:length(varargin)
                switch(varargin{ii})
                    case 'DiffDirection'
                        if varargin{ii+1} == 'pos'
                            differential_direction = 1; % x0 - x
                        elseif varargin{ii+1} == 'neg'
                            differential_direction = -1; % x - x0
                        end
                    case 'VirtTrajMethod'
                        if strcmp(varargin{ii+1}, 'spline')
                            virtual_trajectory_method = 1;
                        elseif strcmp(varargin{ii+1}, 'static')
                            virtual_trajectory_method = 2;
                            nb_samp_avg = 25;
                        elseif strcmp(varargin{ii+1}, 'filter')
                            virtual_trajectory_method = 3;
                            dt = 1e-3; % 1ms
                        elseif strcmp(varargin{ii+1}, 'filterPlus')
                            virtual_trajectory_method = 4;
                            dt = 1e-3; % 1ms
                        elseif strcmp(varargin{ii+1}, 'manual')
                            virtual_trajectory_method = 5;
                            dt = 1e-3; % 1ms
                        else
                            error('Unknown Virtual trajectory method.')
                        end
                    case 'NbSampAvg' % specify nb of samples for the
                        %virtual trajectery estimation before pert.
                        tmp_val = varargin{ii+1};
                        if isnumeric(tmp_val) 
                            nb_samp_avg = floor(varargin{ii+1});
                        else
                            warning('The number of sample for the ' +...
                                'computation of the average should be a ' +...
                                'numeric value. Default value was attributed.')
                            nb_samp_avg = 25;
                        end
                end
            end
        else
            virtual_trajectory_method = 1;
        end

        if virtual_trajectory_method == 3 % filtered signal
            df = designfilt('lowpassfir','PassbandFrequency',8,...
            'StopbandFrequency',8.5,'StopbandAttenuation',20,...
            'SampleRate',1/dt);
            mean_delay = floor(mean(grpdelay(df)));                
            filtered_signal = filter(df, self.complete_traject);
            filtered_signal = circshift(filtered_signal,-mean_delay);
            filtered_signal(end-mean_delay:end) = NaN;

        elseif virtual_trajectory_method == 4 % filtered signal
            filtered_signal = computeVirtualForce(self.time, ...
                self.complete_traject, self.pert_ind, self.interp_window);
        end

        for ii = 1:self.nb_traject

            idx = self.pert_ind(ii);  
            % Real chuncked trajectory
            self.traject(:,ii) = self.complete_traject(idx-2+self.delay:...
                idx+self.estim_window+1+self.delay);
            self.t_traject(:,ii) = self.time(idx-2+self.delay:...
                idx+self.estim_window+1+self.delay);

            % Virtual chuncked trajectory 
            if virtual_trajectory_method == 1 % spline

                self.virt_traject(1:self.interp_window+4,ii) = interp1(...
                    self.t_traject([1,2,3,self.interp_window+2,self.interp_window+3,self.interp_window+4],ii), ...
                    self.traject([1,2,3,self.interp_window+2,self.interp_window+3,self.interp_window+4],ii), ...
                    self.t_traject(1:self.interp_window+4,ii), 'spline');

                if self.estim_window ~= self.interp_window
                    self.virt_traject(self.interp_window+5:end,ii) =...
                        self.traject(self.interp_window+5:end,ii);
                end

            elseif virtual_trajectory_method == 2 % static

                self.virt_traject(:,ii) = mean(self.complete_traject(...
                    idx-2-nb_samp_avg+self.delay:idx-2+self.delay))*...
                    ones(size(self.virt_traject(:,ii)));

            elseif virtual_trajectory_method == 3 % Using filtered signal as ref
                % TODO: deal with the case where:
                % idx+self.estim_window+1+self.delay is within the circ
                % shift and therefore is populated with some NaN
                self.virt_traject(:,ii) = filtered_signal(idx-2+self.delay:...
                idx+self.estim_window+1+self.delay);
            elseif virtual_trajectory_method == 4
                self.virt_traject(:,ii) = filtered_signal(idx-2+self.delay:...
                idx+self.estim_window+1+self.delay);
            elseif virtual_trajectory_method == 5
                % The virtual trajectory was manually fed
                if isnan(self.virt_traject(:,ii))
                    warning("The virtual trajectory " + string(ii) + ...
                        " still contains NaN. The data might have been"...
                        +" fed improperly.")
                end
            end
            % difference between real and virtual
            self.tmp_diff_traject(:,ii) = (self.virt_traject(:,ii) - ...
                self.traject(:,ii))*differential_direction;
            self.diff_traject(:,ii) = self.tmp_diff_traject(3:end-2,ii);

        end

    end

    function self = computeDerivatives(self)
        %addpath('../../../youBot_analysis/Utils');
        for ii = 1:self.nb_traject

            temp_velocity = Iu_diffcent(self.tmp_diff_traject(:,ii), self.t_traject(:,ii));
            temp_acceleration = Iu_diffcent(temp_velocity, self.t_traject(:,ii));

            self.d_diff_traject(:,ii) = temp_velocity(3:end-2);
            self.dd_diff_traject(:,ii) = temp_acceleration(3:end-2);

        end

    end

end
    
end

%Iu_DIFFCENT	derivee numérique par difference centrale
%	Copyright (c) 1994 by M. Gautier, LAN Robotique
%	Exemple : yd=diffcent(y,pas);

function [yd] = Iu_diffcent(y,t)

ny=length(y);

dtemps = [(t(2)-t(1));(t(3:ny)-t(1:ny-2))/2;(t(ny)-t(ny-1))];
yd=[(y(2)-y(1));(y(3:ny)-y(1:ny- 2))/2;(y(ny)-y(ny-1))]./dtemps;
end