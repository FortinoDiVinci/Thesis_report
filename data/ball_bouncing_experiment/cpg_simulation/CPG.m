classdef CPG < handle
    %CPG_CLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Ar;
        Pt;
        y;
        dy; %first derivative
        x1;
        x2;
        dx1; %first derivative
        dx2; %first derivative
        f1;
        f2;
        input;
        alpha;
        c1;
        c2;
        w;
        b;
        t_s;
        t;
    end
    
    methods
        function self = CPG(Ar, Pt, x1, x2, f1, f2, t_s, t)
            self.Ar = Ar;
            self.Pt = Pt;
            self.x1 = x1;
            self.x2 = x2;
            self.f1 = f1;
            self.f2 = f2;
            self.t_s = t_s;
            self.t = t;
            self.c1 = 0.1369214454;
            self.c2 = 0.314366486;
            self.w = 1.688879068736504;
            self.b = 2.511572959291713;
            self.input = 0;
            self.alpha = 10;
        end
        
        function self = matsuoka_output(self)
            [t_int, y] = ode23(@self.matsuoka_diff_equ, [self.t, self.t+self.t_s], [self.x1, self.x2, self.f1, self.f2, self.input]);
            self.x1 = y(end,1);
            self.x2 = y(end,2);
            self.f1 = y(end,3);
            self.f2 = y(end,4);
            self.y = max(self.x1, 0) - max(self.x2, 0); 
            % analytical derivative (using smooth maximum)
%             self.dy = (self.dx1*exp(2*self.alpha*self.x1) + self.dx1*exp(self.alpha*self.x1) + ...
%                 self.x1*self.alpha*self.dx1*exp(self.alpha*self.x1)) / ...
%                 (exp(self.alpha*self.x1) + 1)^2 - ...
%                 (self.dx2*exp(2*self.alpha*self.x2) + self.dx2*exp(self.alpha*self.x2) + ...
%                 self.x2*self.alpha*self.dx2*exp(self.alpha*self.x2)) / ...
%                 (exp(self.alpha*self.x2) + 1)^2;
            % analytical derivative using cases
            if self.x1 >= 0 && self.x2 >= 0
                self.dy = self.dx1 - self.dx2;
            elseif self.x1 >= 0 && self.x2 < 0
                self.dy = self.dx1;
            elseif self.x1 < 0  && self.x2 < 0 
                self.dy = 0;
            elseif self.x1 < 0 && self.x2 >= 0
                self.dy = -self.dx2;
            end
            self.t = t_int(1) + self.t_s;
        end
        
        function dy = matsuoka_diff_equ(self, t, y)
            dy = zeros(5,1); %% derivatives
            % x1 = y(1), x2 = y(2), f1 = y(3), f2 = y(4)
            % input = y(5)
            Tr = self.Pt*self.c1;
            Ta = self.Pt*self.c2;
            
            % parameters for neuron 1
            s1 = self.Ar;
            a21 = self.w;
            Tr1 = Tr;
            Ta1 = Ta;
            b1 = self.b;
            
            % paramters for neuron 2
            s2 = self.Ar;
            a12 = self.w;
            Tr2 = Tr;
            Ta2 = Ta;
            b2 = self.b;            
            
            dy(1) = (-self.x1 - b1*self.f1 - a12*max(self.x2, 0) - max(self.input, 0) + s1)/Tr1;
            dy(2) = (-self.x2 - b2*self.f2 - a21*max(self.x1, 0) - max(-self.input, 0) + s2)/Tr2;
            dy(3) = (-self.f1 + max(self.x1, 0))/Ta1;
            dy(4) = (-self.f2 + max(self.x2, 0))/Ta2;
            dy(5) = 0;
            
            self.dx1 = dy(1);
            self.dx2 = dy(2);
        end      
    end
    
end

