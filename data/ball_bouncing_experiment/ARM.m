classdef ARM < handle
    %ARM_CLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        K;          % stiffness
        B;          % damping
        I;          % inertia
        C;          % Coriolis effect
        G;          % gravity
        Tau;        % torque
        pos;        % height along z axis
        speed;      % speed along z axis
        acc;        % acceleration along z axis
        c_pos;      % commanded position
        c_speed;    % commanded speed
        c_acc;      % commanded acceleration
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
            self.C = 0.5;
            self.G = 9.81 * 1.6;
            self.c_pos = 0;
            self.c_speed = 0;
            self.c_acc = 0;
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
            
            %dy(1) = y(2);
            %dy(2) = (-self.B*y(2) - self.K*y(1) + y(3))/self.I;
            dy(1) = self.speed; % derivative of position            
            dy(2) = (-self.B*self.speed - self.K*self.pos + self.Tau)/self.I; % impedance 2nd order equation
            % dy(3) = 0; % torque is considered constant
        end
        
        function tee_arm_model(self, q_p)
            last_c_speed = self.c_speed;
            self.c_speed = (q_p - self.c_pos)/self.t_s;
            self.c_acc = (self.c_speed - last_c_speed)/self.t_s;
            self.c_pos = q_p;
            
            % feedforward
            tau_ff = self.I*self.c_acc + self.C*self.c_speed + self.G;
            
            % impedance model (feedback)
            tau_fb = self.K*(self.c_pos - self.pos) + self.B*(self.c_speed - self.speed);
            
            tau = tau_ff + tau_fb - self.Tau;
            
            self.acc = (tau - self.G - self.C*self.speed) / self.I;
            self.speed = self.acc*self.t_s + self.speed;
            self.pos = self.speed*self.t_s + self.pos;
            
        end
        
        function torque_disturbance(self, p)
            self.Tau = self.Tau + p;
        end
        
        function position_disturbance(self, p)
            self.pos = self.pos + p;
        end
        
    end
    
end

