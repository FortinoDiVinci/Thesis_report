classdef IMPEDANCE_DATA < handle
    
    % df = K dp + B dv + I da
    % dp position differential
    % dv velocity differential
    % da acceleration differential
    % df force differential
    
    % xi = [K B I]'
    
    % y = xi phi + rho
    
    properties
        
        nb_param; % number of parameters for the impedance model
        phi;      % kinematic data
        y;        % force data, output of the impedance model        
        xi;       % identified impedance parameters
        
        nb_id;    % number of identification to be achieved
        id_size;  % size of the window use for identification
        
        rec_y;    % reconstructed force using identified parameters
        rec_err;  % error between reconstruction and real force
        rec_err_norm;  % normalized error according to force magnitude
        r_2; % coefficient of determination
        rel_std;
        %r_2_fit; % idem but computed with fitlm
        
    end
    
    methods
        
        function self = IMPEDANCE_DATA(nb_parameters, nb_identification, identification_size)
            
            self.nb_param = nb_parameters;
            self.nb_id    = nb_identification;
            self.id_size  = identification_size;
            
            self.xi = NaN(self.nb_param+1, self.nb_id);
            self.phi = zeros(self.id_size, self.nb_param+1, self.nb_id);   
            self.y = NaN(self.id_size, self.nb_id);
            
            self.rec_y = NaN(self.id_size, self.nb_id);
            self.rec_err = NaN(self.id_size, self.nb_id);
            self.rec_err_norm = NaN(self.id_size, self.nb_id);
            self.r_2 = NaN(1,self.nb_id);
            self.rel_std = NaN(self.nb_param,self.nb_id);
            %self.r_2_fit = NaN(self.nb_id,1);
            
        end
        
        function self = init_phi(self, delta_z, delta_dz, delta_ddz)
            
            % argument error management
            switch nargin
                case 2
                    if self.nb_param ~= 1
                        error('' + string(self.nb_param) + ' parameters were expected.' ...
                            + '\n' + string(nargin-1) + ' were provided')
                        return
                    end
                case 3
                    if self.nb_param > 2
                        error('' + string(self.nb_param) + ' parameters were expected.' ...
                            + '\n' + string(nargin-1) + ' were provided')
                        return
                    end
                case 4
                    if self.nb_param > 3
                        error('' + string(self.nb_param) + ' parameters were expected' ...
                            + '\n' + string(nargin-1) + ' were provided')
                        return
                    end
                otherwise
                    error('' + string(self.nb_param) + ' parameters were expected.' ...
                            + '\n' + string(nargin-1) + ' were provided')
            end
            
            dimensions = size(delta_z);
            
            if dimensions(1) == self.id_size && dimensions(2) == self.nb_id
                % data is correctly provided
            elseif dimensions(2) == self.id_size && dimensions(1) == self.nb_id
                % data needs to be transposed
                delta_z = delta_z';
                delta_dz = delta_dz';
                delta_ddz = delta_ddz';
            else
                warning('Input dimensions are not correct, or the number of identification and/or the identification size were not set correctly.')% \n' ... 
                %+ 'Identification size: ' + string(self.id_size) + ', number of identifications: ' + string(self.nb_id))
            end
            
            switch self.nb_param
                case 1
                    for ii = 1:self.nb_id
                        self.phi(:,:,ii) = [delta_z(1:self.id_size,ii), ones(size(delta_z(1:self.id_size,ii)))];
                    end
                case 2
                    for ii = 1:self.nb_id
                        self.phi(:,:,ii) = [delta_z(1:self.id_size,ii), delta_dz(1:self.id_size,ii), ones(size(delta_z(1:self.id_size,ii)))];
                    end
                case 3
                    for ii = 1:self.nb_id
                        self.phi(:,:,ii) = [delta_z(1:self.id_size,ii), delta_dz(1:self.id_size,ii), delta_ddz(1:self.id_size,ii), ones(size(delta_z(1:self.id_size,ii)))];
                    end
                otherwise
                    warning('This number of parameters is not implemented')
            end  
            
        end
        
        function self = init_y(self, delta_fz)
            
            dimensions = size(delta_fz);
            
            if dimensions(1) == self.id_size && dimensions(2) == self.nb_id
                % data is correctly provided
            elseif dimensions(2) == self.id_size && dimensions(1) == self.nb_id
                delta_fz = delta_fz';
            else
                warning('Unsuitable size of the force vector.')
            end
    
            self.y = delta_fz;
            
        end
        
        % least square optimization method with error evaluations
        function self = lsq(self)
            
            for ii = 1:self.nb_id   
                
                % self.xi(:, ii) = self.phi(:,:,ii)\self.y(:, ii);
                self.xi(:, ii) = (self.phi(:,:,ii)'*self.phi(:,:,ii))\self.phi(:,:,ii)'*self.y(:, ii);
                % automated linear fit (brings the same results)
                % mdl = fitlm(self.phi(:,:,ii),self.y(:, ii));
                % self.r_2_fit(ii) = mdl.Rsquared.Adjusted;
                
%                 self.rec_y(:,ii) = self.phi(:,:,ii)*self.xi(:, ii);
%                 self.rec_err(:,ii) = self.rec_y(:,ii) - self.y(:,ii);
%                 
%                 % determination coefficient 
%                 self.r_2(ii) = 1 - sum( self.rec_err(:,ii).^2 ) / ...
%                     sum( (self.y(:, ii) - mean(self.y(:, ii))).^2 );
%                 % relative standard deviation Khalil (2004) eq 12.7 - 12.10
%                 sig_p2 = ( norm(self.rec_err(:,ii))^2 )/ ...
%                     (self.id_size - self.nb_param);
%                 for j = 1:self.nb_param
%                     C = sig_p2*inv(self.phi(:,:,ii)'*self.phi(:,:,ii));
%                     sig_j = sqrt(C(j,j));
%                     self.rel_std(j, ii) = sig_j/abs(self.xi(j,ii));
%                 end
                
            end  
            
            errorStat(self);
            
        end
        
        % the lsq method call this method
        % this method 
        function self = errorStat(self)
            
            for ii = 1:self.nb_id
                self.rec_y(:,ii) = self.phi(:,:,ii)*self.xi(:, ii);
                self.rec_err(:,ii) = self.rec_y(:,ii) - self.y(:,ii);
                
                % determination coefficient 
                self.r_2(ii) = 1 - sum( self.rec_err(:,ii).^2 ) / ...
                    sum( (self.y(:, ii) - mean(self.y(:, ii))).^2 );
                % relative standard deviation Khalil (2004) eq 12.7 - 12.10
                sig_p2 = ( norm(self.rec_err(:,ii))^2 )/ ...
                    (self.id_size - self.nb_param);
                for j = 1:self.nb_param
                    C = sig_p2*inv(self.phi(:,:,ii)'*self.phi(:,:,ii));
                    sig_j = sqrt(C(j,j));
                    self.rel_std(j, ii) = sig_j/abs(self.xi(j,ii));
                end
            end
            
        end
        
    end
    
end