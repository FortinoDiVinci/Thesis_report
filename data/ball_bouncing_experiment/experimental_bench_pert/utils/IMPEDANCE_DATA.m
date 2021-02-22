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
    rmse;
    nrmse;
    %r_2_fit; % idem but computed with fitlm

    arx_id;   % if arx identification was used, this will be fed
    rec_pos;
    rec_pos_err;
    nrmse_pos;
end

methods

    function self = IMPEDANCE_DATA(nb_parameters, nb_identification, identification_size)

        switch (nargin)
            case 0
                % default constructor
                self.xi = [];
                self.phi = [];   
                self.y = [];

                self.rec_y = [];
                self.rec_err = [];
                self.rec_err_norm = [];
                self.r_2 = [];
                self.rmse = [];
                self.rel_std = [];
                %
                self.rec_pos = [];
                self.rec_pos_err = [];
            case 3
        
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
                self.rmse = NaN(1,self.nb_id);
                self.rel_std = NaN(self.nb_param,self.nb_id);
                %self.r_2_fit = NaN(self.nb_id,1);
                self.rec_pos = NaN(self.id_size, self.nb_id);
                self.rec_pos_err = NaN(self.id_size, self.nb_id);
            otherwise
                error('Wrong number of arguments')
        end

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
                    self.phi(:,:,ii) = [delta_z(1:self.id_size,ii), ...
                        ones(size(delta_z(1:self.id_size,ii)))];
                end
            case 2
                for ii = 1:self.nb_id
                    self.phi(:,:,ii) = [delta_z(1:self.id_size,ii), ...
                        delta_dz(1:self.id_size,ii), ones(size(delta_z(1:self.id_size,ii)))];
                end
            case 3
                for ii = 1:self.nb_id
                    self.phi(:,:,ii) = [delta_z(1:self.id_size,ii), ...
                        delta_dz(1:self.id_size,ii), delta_ddz(1:self.id_size,ii),...
                        ones(size(delta_z(1:self.id_size,ii)))];
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
    function self = lsq(self, varargin)
        
        rm_offset = 0;
        ic_zero = 0;
        
        if (~isempty(varargin))
            for c=1:length(varargin)
                switch varargin{c}
                    case {'RmOffsets'}
                        rm_offset = 1;
                    case {'NulInitialCond'}
                        ic_zero = 1;
                        rm_offset = 0;
                otherwise         
                    error(['Invalid optional argument, ', ...
                        varargin{c}]);
                end
            end
        end

        for ii = 1:self.nb_id  
            
            % self.xi(:, ii) = self.phi(:,:,ii)\self.y(:, ii);
            if rm_offset
                phi_avg = self.phi(:,:,ii);
                phi_avg(:,1) = phi_avg(:,1) - mean(phi_avg(:,1));
                y_avg = self.y(:, ii);
                y_avg = y_avg - mean(y_avg);
                self.xi(:, ii) = (phi_avg'*phi_avg)\phi_avg'*y_avg;                
                errorStat(self,'RmOffsets');
            elseif ic_zero
                phi_0 = self.phi(:,:,ii);
                phi_0(:,1) = phi_0(:,1) - phi_0(1,1);
                y_0 = self.y(:, ii);
                y_0 = y_0 - y_0(1);
                self.xi(:, ii) = (phi_0'*phi_0)\phi_0'*y_0;                
                errorStat(self, 'NulInitialCond');
            else
                self.xi(:, ii) = (self.phi(:,:,ii)'*self.phi(:,:,ii))\...
                    self.phi(:,:,ii)'*self.y(:, ii);               
                errorStat(self);
            end
            % automated linear fit (brings the same results)
            % mdl = fitlm(self.phi(:,:,ii),self.y(:, ii));
            % self.r_2_fit(ii) = mdl.Rsquared.Adjusted;
        end  

        

    end

    % The discrete equivalent transfert function to the KBM model is given
    % by: H(z) = Kh*(z + b0)/(z^2 + a1*z + a0)
    % The ARX method helps identifying the coeff, that then need to be
    % identified to extract K, B and M.
    function self = arx(self, varargin)
        
        rm_offset = 0;
        ic_zero = 0;
        
        if (~isempty(varargin))
            for c=1:length(varargin)
                switch varargin{c}
                    case {'RmOffsets'}
                        rm_offset = 1;
                    case {'NulInitialCond'}
                        ic_zero = 1;
                        rm_offset = 0;
                otherwise         
                    error(['Invalid optional argument, ', ...
                        varargin{c}]);
                end
            end
        end

        data = cell(self.nb_id,1);
        self.arx_id = cell(self.nb_id,1);
        dt = 1e-3; % TODO: make it a parameter
        if self.nb_param ~= 3
            error("This method was only implemented for 3 parameters.");
        end
            
        for ii = 1:self.nb_id   
            if rm_offset
                % the signals averages are substracted for the identification
                data{ii} = iddata(self.phi(:,1,ii)-mean(self.phi(:,1,ii)), ...
                    self.y(:,ii)-mean(self.y(:,ii)),dt);
            elseif ic_zero
                data{ii} = iddata(self.phi(:,1,ii) - self.phi(1,1,ii), ...
                    self.y(:,ii) - self.y(1,ii),dt);
            end
            data{ii}.TimeUnit = 's';
            data{ii}.InputUnit = 'N';
            data{ii}.OutputUnit = 'm';                 
        end
        opts = arxOptions('InitialCondition', 'zero');
        for ii = 1:self.nb_id 
            if ic_zero
                self.arx_id{ii} = arx(data{ii},[2 2 0], opts);
            else
                self.arx_id{ii} = arx(data{ii},[2 2 0]);
            end
            self.arx_id{ii}.Name = 'Arx ID';
        end
        syms Ks Bs Ms
        assume(Ks, 'real')
        assume(Bs, 'real')
        assume(Ms, 'real')
        a0 = self.arxCoeffA0(Bs,Ms); % a1 = A1/A2
        a1 = self.arxCoeffA1(Ks,Bs,Ms); % a0 = A0/A2
        b = self.arxCoeffsB(Ks,Bs,Ms); % Kh(1 + b0) = (B1 + B0)/A2
        for ii = 1:self.nb_id 
            eq.A(1) = a1 - self.arx_id{ii}.A(2)/self.arx_id{ii}.A(1) == 0;
            eq.A(2) = a0 - self.arx_id{ii}.A(3)/self.arx_id{ii}.A(1) == 0;
            eq.B(1) = b - (self.arx_id{ii}.B(1) + self.arx_id{ii}.B(2))/...
                self.arx_id{ii}.A(1) == 0;
            sol = vpasolve([eq.A(1),eq.A(2),eq.B(1)],[Ks;Bs;Ms]);
            self.xi(1, ii) = double(sol.Ks);
            self.xi(2, ii) = double(sol.Bs);
            self.xi(3, ii) = double(sol.Ms);
            self.xi(4, ii) = double(0);
        end
        
        if rm_offset
            errorStat(self, 'RmOffsets');
        elseif ic_zero
            errorStat(self, 'NulInitialCond');
        end
        
    end

    function a0 = arxCoeffA0(self,Bs,Ms)
        a0 = exp((Bs.*(-1.0./1.0e+3))./Ms);
    end
    function a1 = arxCoeffA1(self,Ks,Bs,Ms)
        t2 = Bs.^2;
        t3 = 1.0./Ms;
        t4 = Ks.*Ms.*4.0;
        t5 = -t4;
        t6 = t2+t5;
        t7 = sqrt(t6);
        a1 = -(exp(Bs.*t3.*(-1.0./1.0e+3)).*(Ks.*exp((t3.*(Bs-t7))./2.0e+3)...
            +Ks.*exp((t3.*(Bs+t7))./2.0e+3)))./Ks;
    end
    % Kh(1 + b0)
    function b = arxCoeffsB(self,Ks,Bs,Ms)
        t2 = Bs.^2;
        t3 = 1.0./Ms;
        t4 = Ks.*Ms.*4.0;
        t5 = -t4;
        t7 = (Bs.*t3)./1.0e+3;
        t6 = t2+t5;
        t9 = exp(t7);
        t8 = sqrt(t6);
        t10 = Bs+t8;
        t11 = -t8;
        t15 = t8.*t9.*2.0;
        t12 = Bs+t11;
        t13 = (t3.*t10)./2.0e+3;
        t18 = -t15;
        t14 = exp(t13);
        t17 = (t3.*t12)./2.0e+3;
        t16 = Bs.*t14;
        t19 = exp(t17);
        t22 = t8.*t14;
        t20 = Bs.*t19;
        t23 = t8.*t19;
        t21 = -t20;
        t24 = t16+t18+t21+t22+t23;
        b = (t24.*((t8.*-2.0-t16+t20+t22+t23)./t24+1.0).*(-1.0./2.0))./...
            (Ks.*t8.*t9);
    end

    function self = errorStat(self,varargin)
        
        rm_offset = 0;
        ic_zero = 0;
        if (~isempty(varargin))
            for c=1:length(varargin)
                switch varargin{c}
                    case {'RmOffsets'}
                        rm_offset = 1;
                    case {'NulInitialCond'}
                        ic_zero = 1;
                        rm_offset = 0;
                otherwise         
                    error(['Invalid optional argument, ', ...
                        varargin{c}]);
                end
            end
        end
        
        for ii = 1:self.nb_id
            if rm_offset
                phi_avg0 = self.phi(:,:,ii);
                phi_avg0(:,1) = phi_avg0(:,1) - mean(phi_avg0(:,1));
                self.rec_y(:,ii) = phi_avg0*self.xi(:, ii);
                output = self.y(:,ii) - mean(self.y(:,ii));
            elseif ic_zero
                % initial position is shifted to be null
                phi_ic0 = self.phi(:,:,ii);
                phi_ic0(:,1) = phi_ic0(:,1) - phi_ic0(1,1);
                self.rec_y(:,ii) = phi_ic0*self.xi(:, ii);
                output = self.y(:,ii) - self.y(1,ii);
            else
                self.rec_y(:,ii) = self.phi(:,:,ii)*self.xi(:, ii);
                output = self.y(:,ii);
            end
            self.rec_err(:,ii) = self.rec_y(:,ii) - output;
            % root mean square error 
            self.rmse(ii) = sqrt(mean(self.rec_err(:,ii).^2));
            self.nrmse(ii) = 1 - norm(self.rec_err(:,ii)) / ...
                norm(mean(output) - output);
            % determination coefficient 
            self.r_2(ii) = 1 - sum( self.rec_err(:,ii).^2 ) / ...
                sum( (output - mean(output)).^2 );
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

    % the impedance system in the causal form should consider the force
    % as the input and the position as the output. For more details, have 
    % a look on the live script impedance_identification.mlx
    function self = causalSim(self, dt, varargin)
        
        rm_offset = 0;
        ic_zero = 0;
        if (~isempty(varargin))
            for c=1:length(varargin)
                switch varargin{c}
                    case {'RmOffsets'}
                        rm_offset = 1;
                    case {'NulInitialCond'}
                        ic_zero = 1;
                        rm_offset = 0;
                otherwise         
                    error(['Invalid optional argument, ', ...
                        varargin{c}]);
                end
            end
        end
        
        for ii = 1:self.nb_id
            A = [  0 ,   1 ;
            -self.xi(1,ii)/self.xi(3,ii), -self.xi(2,ii)/self.xi(3,ii)];
            B = [0; 1/self.xi(3,ii)];
            C = [1, 0];
            causal_ss = ss(A,B,C,0);
            if rm_offset
                input = self.y(:,ii)-mean(self.y(:,ii));
                output = self.phi(:,1,ii) - mean(self.phi(:,1,ii));
                ic = [output(1), self.phi(1,2,ii)];
            elseif ic_zero
                input = self.y(:,ii)-self.y(1,ii);
                output = self.phi(:,1,ii) - self.phi(1,1,ii);
                ic = [output(1), self.phi(1,2,ii)];
            else
                input = self.y(:,ii);
                output = self.phi(:,1,ii);
                ic = [output(1), self.phi(1,2,ii)];
            end
            self.rec_pos(:,ii) = lsim(causal_ss, input, dt*(0:199), ic);
            self.rec_pos_err(:,ii) = self.rec_pos(:,ii) - output;
            self.nrmse_pos(ii) = 1 - norm(self.rec_pos_err(:,ii))/...
                norm(mean(output) - output);
        end
    end
    
end
    
end