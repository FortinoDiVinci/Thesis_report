classdef CPG < handle
    %CPG_CLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Ar;
        Pt;
        y_out;
        x1_out;
        x2_out;
        f1_out;
        f2_out;
        input;
        c1;
        c2;
        w;
        b;
        t_s;
        t;
    end
    
    methods
        function self = CPG(Ar, Pt, x1_out, x2_out, f1_out, f2_out, t_s, t)
            self.Ar = Ar;
            self.Pt = Pt;
            self.x1_out = x1_out;
            self.x2_out = x2_out;
            self.f1_out = f1_out;
            self.f2_out = f2_out;
            self.t_s = t_s;
            self.t = t;
            self.c1 = 0.1369214454;
            self.c2 = 0.314366486;
            self.w = 1.688879068736504;
            self.b = 2.511572959291713;
            self.input = 0;
        end
        
        function self = matsuoka_output(self)
            [t_int, y] = ode23(@self.matsuoka_diff_equ, [self.t, self.t+self.t_s], [self.x1_out, self.x2_out, self.f1_out, self.f2_out, self.input]);
            self.x1_out = y(end,1);
            self.x2_out = y(end,2);
            self.f1_out = y(end,3);
            self.f2_out = y(end,4);
            self.y_out = max(self.x1_out, 0) - max(self.x2_out, 0); 
            self.t = t_int(1) + self.t_s;
        end
        
        function dy = matsuoka_diff_equ(self, t, y)
            dy = zeros(5,1); %% derivatives
            % x1_out = y(1), x2_out = y(2), f1_out = y(3), f2_out = y(4)
            % input = y(5)
            Tr = self.Pt * self.c1;
            Ta = self.Pt * self.c2;
            
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
            
            dy(1) = (-self.x1_out - a12*max(self.x2_out, 0) + s1 - b1*self.f1_out - max(self.input, 0))/Tr1;
            dy(2) = (-self.x2_out - a21*max(self.x1_out, 0) + s2 - b2*self.f2_out - max(-self.input, 0))/Tr2;
            dy(3) = (-self.f1_out + max(self.x1_out, 0))/Ta1;
            dy(4) = (-self.f2_out + max(self.x2_out, 0))/Ta2;
            dy(5) = 0;
        end      
    end
    
end

