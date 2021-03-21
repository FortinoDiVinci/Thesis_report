syms a1 a0 sum_b

lambda1 = (-a1 + sqrt(a1^2 - 4*a0))/2;
lambda2 = (-a1 - sqrt(a1^2 - 4*a0))/2;
dt = 1e-3;
K = (1 + a1 + a0)/(sum_b);
M = -K*dt^2/(log(lambda1)*(log(lambda1) - log(a0)));
B = -log(a0)*M/dt;

a0i = 0.9900;
a1i = -1.9899;
sum_bi = 9.9500e-07;

sensK = double(subs(jacobian(K,[a0,a1,sum_b]), [a0,a1,sum_b], [a0i,a1i,sum_bi]));
sensM = double(subs(jacobian(M,[a0,a1,sum_b]), [a0,a1,sum_b], [a0i,a1i,sum_bi]));
sensB = double(subs(jacobian(B,[a0,a1,sum_b]), [a0,a1,sum_b], [a0i,a1i,sum_bi]));


figure('DefaultAxesFontSize',14)
bar(abs([sensK;sensB;sensM]))
set(gca,'YScale','log')
set(gca,'xticklabel',{'K';'B';'M'})
title('Sensitivity analysis')
legend('a0', 'a1', 'b0+b1')
ylabel('Sensitivity')