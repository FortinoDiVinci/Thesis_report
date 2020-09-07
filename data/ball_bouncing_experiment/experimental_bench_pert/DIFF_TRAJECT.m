classdef DIFF_TRAJECT < handle

    
    properties
        
        estim_window;  % number of sample for the desired trajectories
        interp_window; % number of samples for the interpolation 
        nb_traject;    % number of trajectories to be estimated
        
        pert_ind;      % indexes at which trajectories will be estimated
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
        
        function self = DIFF_TRAJECT(estimation_window_size, interpolation_window_size, complete_trajectory, time_frame, perturbation_indexes, delay_sample)
            
            self.interp_window = interpolation_window_size;
            self.estim_window = estimation_window_size;
            self.nb_traject = length(perturbation_indexes);
            
            self.complete_traject = complete_trajectory;
            self.time = time_frame;
            self.pert_ind = perturbation_indexes;
            self.delay = delay_sample;

            self.t_traject = NaN(self.estim_window + 4, self.nb_traject);
            self.traject = NaN(self.estim_window + 4, self.nb_traject);            
            self.virt_traject = NaN(self.estim_window + 4, self.nb_traject);
            self.tmp_diff_traject = NaN(self.estim_window + 4, self.nb_traject);
            
            self.diff_traject = NaN(self.estim_window, self.nb_traject); 
            self.d_diff_traject = NaN(self.estim_window, self.nb_traject); 
            self.dd_diff_traject = NaN(self.estim_window, self.nb_traject); 
        end
        
        function self = computeDiffTraject(self, varargin)
            
            if ~isempty(varargin)
                for ii = 1:2:length(varargin)
                    switch(varargin{ii})
                        case 'VirtTrajMethod'
                            if varargin{ii+1} == 'spline'
                                virtual_trajectory_method = 1;
                            elseif varargin{ii+1} == 'static'
                                virtual_trajectory_method = 2;
                                nb_samp_avg = 25;
                            end
                        case 'NbSampAvg' % specify nb of samples for the
                            %virtual trajectery estimation before pert.
                            tmp_val = varargin{ii+1};
                            if isnumeric(tmp_val) 
                                nb_samp_avg = floor(varargin{ii+1});
                            else
                                warning('The number of sample for the computation of the average should be a numeric value. Default value was attributed.')
                                nb_samp_avg = 25;
                            end
                    end
                end
            else
                virtual_trajectory_method = 1;
            end
            
            if virtual_trajectory_method == 1 % spline
            
                for ii = 1:self.nb_traject
                    
                    idx = self.pert_ind(ii);               
                    self.traject(:,ii) = self.complete_traject(idx-2+self.delay:idx+self.estim_window+1+self.delay);
                    self.t_traject(:,ii) = self.time(idx-2+self.delay:idx+self.estim_window+1+self.delay);

                    self.virt_traject(1:self.interp_window+4,ii) = interp1(self.t_traject([1,2,3,self.interp_window+2,self.interp_window+3,self.interp_window+4],ii), self.traject([1,2,3,self.interp_window+2,self.interp_window+3,self.interp_window+4],ii), self.t_traject(1:self.interp_window+4,ii), 'spline');
                    
                    if self.estim_window ~= self.interp_window
                        self.virt_traject(self.interp_window+5:end,ii) = self.traject(self.interp_window+5:end,ii);
                        %self.tmp_diff_traject(:,ii) = [self.virt_traject(1:self.interp_window+4,ii) - self.traject(1:self.interp_window+4,ii); zeros(size(self.traject(self.interp_window+5:end,ii)))];
                    %else
                        %self.tmp_diff_traject(:,ii) = self.virt_traject(:,ii) - self.traject(:,ii);
                    end
                    self.tmp_diff_traject(:,ii) = self.virt_traject(:,ii) - self.traject(:,ii);
                    self.diff_traject(:,ii) = self.tmp_diff_traject(3:end-2,ii);

                end
            
            elseif virtual_trajectory_method == 2 % static
                
                for ii = 1:self.nb_traject

                    idx = self.pert_ind(ii);               
                    self.traject(:,ii) = self.complete_traject(idx-2+self.delay:idx+self.estim_window+1+self.delay);
                    self.t_traject(:,ii) = self.time(idx-2+self.delay:idx+self.estim_window+1+self.delay);

                    self.virt_traject(:,ii) = mean(self.complete_traject(idx-2-nb_samp_avg+self.delay:idx-2+self.delay))*ones(size(self.virt_traject(:,ii)));

                    self.tmp_diff_traject(:,ii) = self.virt_traject(:,ii) - self.traject(:,ii);
                    self.diff_traject(:,ii) = self.tmp_diff_traject(3:end-2,ii);

                end
                
            end
            
        end
        
        function self = computeDerivatives(self)
            
            for ii = 1:self.nb_traject
                
                temp_velocity = Iu_diffcent(self.tmp_diff_traject(:,ii), self.t_traject(:,ii));
                temp_acceleration = Iu_diffcent(temp_velocity, self.t_traject(:,ii));
                
                self.d_diff_traject(:,ii) = temp_velocity(3:end-2);
                self.dd_diff_traject(:,ii) = temp_acceleration(3:end-2);
                
            end
            
        end
        
    end
    
end