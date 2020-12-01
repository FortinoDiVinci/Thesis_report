%% Compare both virtual trajectory and impedance estimations using 
% different methodologies on the same set
clear all 
close all

load('impedance_res_spline_virt')

data(1).impedance = impedance;
data(1).position = delta_z;
data(1).force = delta_fz;
data(1).method = "spline";
data(1).res.R2 = [];
data(1).res.K = [];
data(1).res.B = [];
data(1).res.M = [];

load('impedance_res_filtered_virt')

data(2).impedance = impedance;
data(2).position = delta_z;
data(2).method = "filter";
data(2).force = delta_fz;
data(2).res.R2 = [];
data(2).res.K = [];
data(2).res.B = [];
data(2).res.M = [];

clearvars -except data

dt = 1e-3;
nb_tot_exp = length(data(1).impedance);

for meth_nb = 1:length(data)
    for ii = 1:nb_tot_exp

        data(meth_nb).res.R2 = [data(meth_nb).res.R2, data(meth_nb).impedance{ii}.r_2'];    
        data(meth_nb).res.K = [data(meth_nb).res.K, data(meth_nb).impedance{ii}.xi(1,:)];
        data(meth_nb).res.B = [data(meth_nb).res.B, data(meth_nb).impedance{ii}.xi(2,:)];
        data(meth_nb).res.M = [data(meth_nb).res.M, data(meth_nb).impedance{ii}.xi(3,:)];

    end
end


figure
hold on
for ii = 1:length(data)
    histogram(data(ii).res.R2, 15, 'Normalization','probability')
end
xlabel('R2 score')
legend(data(1).method, data(2).method)
title('R2 score distribution')

figure
hold on
for ii = 1:length(data)
    histogram(data(ii).res.K, 15, 'Normalization','probability')
end
xlabel('Stiffness')
legend(data(1).method, data(2).method)
title('K distribution')

figure
hold on
for ii = 1:length(data)
    histogram(data(ii).res.B, 15, 'Normalization','probability')
end
xlabel('Damping')
legend(data(1).method, data(2).method)
title('B distribution')

figure
hold on
for ii = 1:length(data)
    histogram(data(ii).res.M, 15, 'Normalization','probability')
end
xlabel('Mass')
legend(data(1).method, data(2).method)
title('M distribution')