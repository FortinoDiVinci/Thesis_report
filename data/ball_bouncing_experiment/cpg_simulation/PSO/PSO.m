% *************************************************************************
% *************************************************************************
% ***                                                                   ***
% ***              Least Square Error optimizartion for                 ***
% ***         the gain of the equilibrium position input h2             ***
% ***                                                                   ***
% *************************************************************************
% *************************************************************************
%
%

clear all;

addpath('../');

lb = [0];                       % lower bound
ub = [1];                        % upper bound
nvars = 1;

t_d = 48.0;                     % Delay upon the perception of the ball
t_s_input = 0.003;              % CPG input entrainment sampling period (s)
lbd = -3.4;                     % Adaptation gain of cpg input
h_0 = 96.54;                    % Sensor input gain


func = @(h_2)bouncing_ball_tee_model(t_d, t_s_input, lbd, h_0, h_2);   

options = optimoptions('particleswarm','SwarmSize',10,'HybridFcn',@fmincon, 'Display','iter', 'MaxStallIterations', 4);
x = particleswarm(func, nvars, lb, ub, options)

