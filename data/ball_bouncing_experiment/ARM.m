classdef ARM < handle
    %ARM_CLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        K;          % stiffness
        B;          % damping
        I;          % inertia
        Tau;        % torque
        pos;        % height along z axis
        speed;      % speed along z axis
        t_s;        % sampling time
        t;          % time
        t_d; %%
    end
    
    methods
        function self = ARM(K, B, I, z, vz, t_s, t)
            self.K = K;
            self.B = B;
            self.I = I;
            self.Tau = 0;
            self.pos = z;
            self.speed = vz;
            self.t_s = t_s;
            self.t = t;
            self.t_d = t;
        end
        
        function self = impedance_output(self)
            [t_int, y] = ode23(@self.impedance_diff_equ, [self.t, self.t+self.t_s], [self.pos, self.speed, self.Tau]);
            self.pos = y(end,1);
            self.speed = y(end,2);
            self.t = t_int(1) + self.t_s;   
            self.t_d = [self.t_d; self.t];
        end
        
        function dy = impedance_diff_equ(self, t, y)
            dy = zeros(3,1); % derivatives

            dy(1) = self.speed; % derivative of position            
            dy(2) = (-self.B*self.speed - self.K*self.pos + self.Tau)/self.I; % impedance 2nd order equation
            % dy(3) = 0; % torque is considered constant
        end
        
        function torque_disturbance(self, p)
            self.Tau = self.Tau + p;
        end
        
        function position_disturbance(self, p)
            self.pos = self.pos + p;
        end
        
    end
    
end

