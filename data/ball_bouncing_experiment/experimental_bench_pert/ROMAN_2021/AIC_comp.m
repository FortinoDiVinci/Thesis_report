clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

%% force traject

load('../trajectory_prediction_evaluation/force_trajectory_optimization/test_sine_opt_5.mat')

BIC = NaN(size(delta_fz,2),1);
AIC = NaN(size(delta_fz,2),1);
AICc = NaN(size(delta_fz,2),1);

for method_i = 1:size(delta_fz,2)  
    fz_data = [delta_fz{2:15,method_i}];
    epsilon = [fz_data(:).diff_traject];

    SSE = sum(sum(epsilon(1:100,:).^2)); % Sum of Squared Errors
    n = size(epsilon,1)*size(epsilon,2); % Number of samples in total
    p = size(fz_data(1).opt_param,1); % Number of parameters (weights and biases)
    % Schwarz's Bayesian criterion (or BIC) (Schwarz, 1978)
    BIC(method_i) = n * log(SSE/n) + p * log(n);
    % Akaike's information criterion (Akaike, 1969)
    AIC(method_i) = n * log(SSE/n) + 2 * p;
    % Corrected AIC (Hurvich and Tsai, 1989)
    AICc(method_i) = n * log(SSE/n) + (n + p) / (1 - (p + 2) / n);
end

figure('DefaultAxesFontSize',14)
hold on
plot(AICc)
plot(AIC)
plot(BIC)
legend('AICc', 'AIC', 'BIC')
xlabel('Method')
ylabel('score')
title('Prediction error estimator')

%% impedance

clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('imp_test_05.mat')

method_i = 11; % 2sineOptM
% delay_i = 6; % 10ms
delay_i = 2; % 10ms

for method_i = 1:11
    imp_data = [data(delay_i).impedance{:,method_i}];
    y = [];
    y_hat = [];
    for exp_nb = 1:length(imp_data)
        y = [y, squeeze(imp_data(exp_nb).phi(:,1,:))];
        y_hat = [y_hat, imp_data(exp_nb).rec_pos];
    end
    
    % Getting the training targets
    SSE = sum(sum((y - y_hat).^2)); % Sum of Squared Errors for the training set
    n = size(y,1)*size(y,2); % Number of training cases
    p = size(data(delay_i).delta_fz{1,method_i}.opt_param,1); % Number of parameters (weights and biases)
    % Schwarz's Bayesian criterion (or BIC) (Schwarz, 1978)
    BIC(method_i) = n * log(SSE/n) + p * log(n);
    % Akaike's information criterion (Akaike, 1969)
    AIC(method_i) = n * log(SSE/n) + 2 * p;
    % Corrected AIC (Hurvich and Tsai, 1989)
    AICc(method_i) = n * log(SSE/n) + (n + p) / (1 - (p + 2) / n);   
end

figure('DefaultAxesFontSize',14)
hold on
plot(AICc)
plot(AIC)
plot(BIC)
legend('AICc', 'AIC', 'BIC')
xlabel('Method')
ylabel('score')
title('Prediction error estimator')