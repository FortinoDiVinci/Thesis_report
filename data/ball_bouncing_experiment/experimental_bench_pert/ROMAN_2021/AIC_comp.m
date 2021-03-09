clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

%% force traject

load('../trajectory_prediction_evaluation/force_trajectory_optimization/test_sine_opt_3.mat')

method_i = 11; % 2sineOptM
fz_data = [delta_fz{2:15,method_i}];
epsilon = [fz_data(:).diff_traject];

SSE = sum(epsilon.^2); % Sum of Squared Errors for the training set
n = length(SSE); % Number of training cases
p = size(fz_data(1).opt_param,1); % Number of parameters (weights and biases)
% Schwarz's Bayesian criterion (or BIC) (Schwarz, 1978)
BIC = n * log(SSE/n) + p * log(n);
% Akaike's information criterion (Akaike, 1969)
AIC = n * log(SSE/n) + 2 * p;
% Corrected AIC (Hurvich and Tsai, 1989)
AICc = n * log(SSE/n) + (n + p) / (1 - (p + 2) / n);

%% impedance

clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('imp_test_05.mat')

method_i = 11; % 2sineOptM
delay_i = 6; % 10ms
imp_data = [data(delay_i).impedance{:,method_i}];
y = [];
y_hat = [];
for exp_nb = 1:length(imp_data)
    y = [y, squeeze(imp_data(exp_nb).phi(:,1,:))];
    y_hat = [y_hat, imp_data(exp_nb).rec_pos];
end

% Getting the training targets
SSE = sum((y - y_hat).^2); % Sum of Squared Errors for the training set
n = length(SSE); % Number of training cases
p = 3*2 + ; % Number of parameters (weights and biases)
% Schwarz's Bayesian criterion (or BIC) (Schwarz, 1978)
SBC = n * log(SSE/n) + p * log(n);
% Akaike's information criterion (Akaike, 1969)
AIC = n * log(SSE/n) + 2 * p;
% Corrected AIC (Hurvich and Tsai, 1989)
AICc = n * log(SSE/n) + (n + p) / (1 - (p + 2) / n);