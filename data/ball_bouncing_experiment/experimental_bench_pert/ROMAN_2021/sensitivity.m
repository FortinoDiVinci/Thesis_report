syms a1 a0 sum_b b0 b1

lambda1 = (-a1 + sqrt(a1^2 - 4*a0))/2;
lambda2 = (-a1 - sqrt(a1^2 - 4*a0))/2;
dt = 1e-3;
K = (1 + a1 + a0)/(b0 + b1);
M = -K*dt^2/(log(lambda1)*(log(lambda1) - log(a0)));
B = -log(a0)*M/dt;

a0i = 0.9900;
a1i = -1.9899;
% sum_bi = 9.9500e-07;
b1i = 9.9333e-7;
b0i = 9.8673e-7;


% normalized?
% a0i = 1;
% a1i = 1;
% b0i = 1;
% b1i = 1;

sensK = double(subs(jacobian(K,[a0,a1,b0,b1]), [a0,a1,b0,b1], [a0i,a1i,b0i,b1i]));
sensM = double(subs(jacobian(M,[a0,a1,b0,b1]), [a0,a1,b0,b1], [a0i,a1i,b0i,b1i]));
sensB = double(subs(jacobian(B,[a0,a1,b0,b1]), [a0,a1,b0,b1], [a0i,a1i,b0i,b1i]));


figure('DefaultAxesFontSize',14)
bar(abs([sensK;sensB;sensM]))
set(gca,'YScale','log')
set(gca,'xticklabel',{'K';'B';'M'})
title('Sensitivity analysis')
legend('a0', 'a1', 'b0', 'b1')
ylabel('Sensitivity')