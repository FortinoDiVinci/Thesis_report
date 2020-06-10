clear all

%% VARIABLES

% PARAMETERS
kb = 15000;     % ball stiffness (N/m)
ka = 500;       % arm stiffness (N/m)
mb = 0.058;     % ball mass (kg)
ma = 1;         % arm mass (kg)
bb = 5;         % ball damping (Ns^-1/m)
ba = 25;        % arm damping (Ns^-1/m)
r  = 0.05;      % ball radius
g  = 9.81;      % gravity acceleration
xv = 0;         % virtual position of the arm
tau= 40;        % force input coefficient
%f  = [];        % force
phi= 6/8*pi;    % input force delay (sin(wt+phi))

% DURATION & TIME
t_free              = 0;    
max_flight_duration = 2;    % 2 s
max_impact_duration = 0.04; % 40 ms
sampling_rate       = 1e-4; % 0.1 ms
t_sim               = 15;   % 15 s

% INITIAL CONDITIONS
xb0 = 0.5;      % m
vb0 = 0;        % m.s^-1
xa0 = 0;        % m
va0 = 0;        % m.s^-1
xa = [];        % arm positions
xb = [];        % ball positions
t  = [];        % simulation time
alpha = [];     % coefficient of restitution
impact_durations = [];

%% SIMULATION

while (isempty(t) || t(end) <= t_sim)

    % FREE MOVEMENT EQUATIONS

    t_interval = [t_free, t_free + max_flight_duration];
    sampling = linspace(t_interval(1),t_interval(end), max_flight_duration/sampling_rate+1);
    t_eval = (t_interval(1):sampling_rate:t_interval(end))';
    sol_xb = ode45(@(t_ode,Y) odeBallisticFcn(t_ode,Y, g) , t_interval , [xb0, vb0]); 
     
    yb = deval(sol_xb, sampling)';
    
    [ball_apex, idx_apex] = max(yb(:,1));
    t_apex = t_eval(idx_apex);
    if t_apex ~= 0
        frequency = 0.5/(t_apex - t_free);  % (f=1/2ta)
    else
        frequency = 1.67;
    end
    
    t_f = t_interval(1):sampling_rate:t_interval(end) + max_impact_duration;
    f = tau*sin(2*pi*frequency*(t_f-t_free) + phi); % arm input force
    %f = zeros(1,length(t_f));
    
    sol_xa = ode45(@(t_ode,Y) odeArmFcn(t_ode,Y, ma,ba,ka, t_f,f) , t_interval , [xa0, va0]);
    ya = deval(sol_xa, sampling)';
    
    % IMPACT DIFFERENTIAL EQUATIONS

    % detects beginning of the impact
    distance = yb(:,1) - ya(:,1);
    idx_start_impact = find(distance <= r, 1, 'first');

%     if ( abs(r-distance(idx_start_impact-1)) < abs(r-distance(idx_start_impact)) )
%         % the point before impact is closer than the one right after
%         idx_start_impact = idx_start_impact - 1; 
%     end

    t_impact = t_eval(idx_start_impact);
    if(isempty(idx_start_impact))
        % end of simulation
        xb = [xb; yb(:,1)];
        xa = [xa; ya(:,1)];
        t  = [t; t_eval];
        disp('New impact not found, the ball was sent too far away')
        break;
    else
        xb = [xb; yb(1:idx_start_impact-1,1)];
        xa = [xa; ya(1:idx_start_impact-1,1)];
        t  = [t;  t_eval(1:idx_start_impact-1)];

        xb0 = yb(idx_start_impact,1);
        vb0 = yb(idx_start_impact,2);
        xa0 = ya(idx_start_impact,1);
        va0 = ya(idx_start_impact,2);

        init_cond = [xb0,xa0,vb0,va0];
        t_interval = [t_impact, t_impact + max_impact_duration];    
        
        % arm input force during impact (no change of intention)
        t_f = t_interval(1):sampling_rate:t_interval(end);
        f = tau*sin(2*pi*frequency*(t_f-t_free) + phi); % arm input force
        %f = zeros(1,length(t_f));
        
        sol_xaxb = ode45(@(t_ode,Y) odeImpactFcn(t_ode,Y, kb,ka,mb,ma,bb,ba,r,g,xv, t_f,f) , t_interval , init_cond);        

        sampling = linspace(t_interval(1),t_interval(end), max_impact_duration/sampling_rate+1);
        y = deval(sol_xaxb, sampling)';
        t_eval = (t_interval(1):sampling_rate:t_interval(end))';

        idx_end_impact = find(y(:,1)-y(:,2) > r, 1, 'first');

        if(isempty(idx_end_impact))
            % end of simulation
            xb = [xb; y(:,1)];
            xa = [xa; y(:,2)];
            t  = [t; t_eval];
            disp('End of impact not found, the ball is not bouncing anymore')
            break;
        else    
            
            xb0 = y(idx_end_impact, 1); % ball position right after impact
            vb0 = y(idx_end_impact, 3); % ball velocity right after impact
            xa0 = y(idx_end_impact, 2); % arm position right after impact
            va0 = y(idx_end_impact, 4); % arm velocity right after impact
  
            xb = [xb; y(1:idx_end_impact-1,1)];
            xa = [xa; y(1:idx_end_impact-1,2)];
            t  = [t;  t_eval(1:idx_end_impact-1)];
            t_free = t_eval(idx_end_impact);           
            
            impact_durations = [impact_durations; t_free-t_impact];
            alpha = [alpha; -(vb0-va0)/(init_cond(3)-init_cond(4))];
            
        end

    end

end
figure
plot(t, xb, 'r', t, xa, 'b')
