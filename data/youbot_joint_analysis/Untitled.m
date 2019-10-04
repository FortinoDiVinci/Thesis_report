p = tf('p')

%kf = 0;

He = 1/(L*p+R);
Hm = 1/(J*p+kf);
mu = He*Kt*Hm;

Homega = feedback(mu, Ke);
Hi = Homega*1/Hm/Kt;
%Hi1 = feedback(He, Kt*Hm*Ke);

% K_cur = 1500;
% Ki_cur = 1500;
% K_cur = 1500/256;
% Ki_cur = 1500/262144;

Ti = 1/(0.5e4);
Ki = 10^(11/20);
% Ki = 10^(-30/20);
K_cur = Ki;
Ki_cur = Ki/Ti;

Cpi = K_cur + Ki_cur/p;

% tm = 5e-4;
% Dphi = 58;
% xi = 0.6;
% 
% wc = pi/sqrt(1-xi^2)/tm;
% 
% [mag,phase,wout] = bode(Hi, wc);
% 
% poly = [L*Jm, L*kf + R*J, R*kf + Kt*Ke];

figure
bode(Hi*Cpi)

figure
step(feedback(Hi*Cpi,1))

figure
step(minreal(Cpi/(1+Cpi*Hi)),1e-3)