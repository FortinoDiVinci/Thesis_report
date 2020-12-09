classdef PI
    %PID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        P;
        I;
        error_sum;
        dt;
        dim; % dimension
    end
    
    methods
        function self = PI(dt, P, I)
            self.dt = dt;
            self.P = P;
            self.I = I;
            self.error_sum = zeros(size(P));
            if(size(P) ~= size(I))
                error('Dimensions of P and I gains are not consistent')
            else
                self.dim = length(P);
            end
        end
        
        function cmd = compute(self, error, limitReached)
            if nargin < 3
                limitReached = zeros(size(self.P));
            end
            for it = 1:self.dim
                if (limitReached(it))
                    self.error_sum(it) = 0;
                else
                    self.error_sum(it) = self.error_sum(it) + error(it);
                end
            end
            cmd = self.P .* error + self.I .* self.error_sum .* self.dt;
        end
    end
end

