classdef ARM < handle
    %ARM_CLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        K;          % stiffness
        B;          % damping
        I;          % inertia
        C;          % Centrifugal & Coriolis effect
        G;          % gravity
        l;          % hand and forearm length
        m;          % hand and forearm mass
        Tau;        % torque
        tau_e;      % dist torque
        tau_ff;     % feedforward computed torque
        tau_fb;     % impedance computed torque
        pos;        % height along z axis
        speed;      % speed along z axis
        acc;        % acceleration along z axis
        c_pos;      % commanded position
        c_speed;    % commanded speed
        c_acc;      % commanded acceleration
        tau_f;      % filtering time cst for acceleration
        t_s;        % sampling time
        t;          % time
        t_d;        % data recording (time)
    end
    
    methods
        function self = ARM(K, B, I, z, vz, az, t_s, t)
            self.K = K;
            self.B = B;
            self.I = I;
            self.l = (0.146 + 0.108)*1.75;
            self.m = (0.019 + 0.006)*70;
            self.Tau = 0;
            self.tau_e = 0;
            self.tau_ff = 0;
            self.tau_fb = 0;
            self.pos = z;
            self.speed = vz;
            self.acc = az;
            self.tau_f = 2*0.03;
            self.t_s = t_s;
            self.t = t;
            self.t_d = t;
            self.C = 0;
            self.G = self.l*self.m*9.81; 
            self.c_pos = 0;
            self.c_speed = 0;
            self.c_acc = 0;
        end
        
        function self = inertial_coupling(self, theta, omega)
            self.C = self.m*omega^2*sin(theta);
        end
        
%         function self = positional_torque(self, theta)
%             self.G = self.l*self.m*9.81;
%         end
        
        function self = impedance_output(self)
            [t_int, y] = ode23(@self.impedance_diff_equ, [self.t, self.t+self.t_s], [self.pos, self.speed, self.Tau]);
            self.pos = y(end,1);
            self.speed = y(end,2);
            self.t = t_int(1) + self.t_s;
            self.t_d = [self.t_d; self.t];
        end
        
        function dy = impedance_diff_equ(self, t, y)
            dy = zeros(3,1); % derivatives           
            dy(1) = y(2);
            dy(2) = (-self.B*y(2) - self.K*y(1) + y(3))/self.I;
            self.acc = dy(2);
            %dy(1) = self.speed; % derivative of position            
            %dy(2) = (-self.B*self.speed - self.K*self.pos + self.Tau)/self.I; % impedance 2nd order equation
            %dy(3) = 0; % torque is considered constant
        end
        
        function tee_arm_model(self, q_p, dq_p)
            last_c_speed = self.c_speed;
            %self.c_speed = (q_p - self.c_pos)/self.t_s;
            %self.c_acc = (self.c_speed - last_c_speed)/self.t_s;
            %self.c_pos = q_p;
            self.c_pos = q_p;
            % filtereing speed
            self.c_speed = (self.t_s/self.tau_f)/(1 + self.t_s/self.tau_f)*dq_p + ...
                1/(1 + self.t_s/self.tau_f)*self.c_speed;
            c_acc = (self.c_speed - last_c_speed)/self.t_s;
            % filtering acceleration
            self.c_acc = (self.t_s/(10*self.tau_f))/(1 + self.t_s/(10*self.tau_f))*c_acc + ...
                1/(1 + self.t_s/(10*self.tau_f))*self.c_acc;
            
            % feedforward
            self.inertial_coupling(q_p, dq_p); % compute self.C 
            self.tau_ff = self.I*self.c_acc + self.C*self.c_speed + self.G;
            %self.tau_ff = 0;
            
            % impedance model (feedback)
            self.tau_fb = self.K*(self.c_pos - self.pos) + self.B*(self.c_speed - self.speed);% + self.I*(self.c_acc - self.acc);
            
            self.Tau = self.tau_ff + self.tau_fb + self.tau_e;
            
            %self.acc = (tau - self.G - self.C*self.speed) / self.I;
            %self.speed = self.acc*self.t_s + self.speed;
            %self.pos = self.speed*self.t_s + self.pos;
            
            [t_int, y] = ode23(@self.impedance_diff_equ, [self.t, self.t+self.t_s], [self.pos, self.speed, self.Tau]);
            self.pos = y(end,1);
            self.speed = y(end,2);
            self.t = t_int(1) + self.t_s;
            self.t_d = [self.t_d; self.t];  
        end
        
        function torque_disturbance(self, p)
            self.tau_e = p;
        end
        
        function position_disturbance(self, p)
            self.pos = self.pos + p;
        end
        
    end
    
end

