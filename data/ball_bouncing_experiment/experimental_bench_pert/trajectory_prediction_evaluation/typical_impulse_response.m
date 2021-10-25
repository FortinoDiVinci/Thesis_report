K = 539;
B = 44;
M  = 2.8;

K = 280;
B = 12;
M = 0.6;

xi = B/sqrt(4*M*K);
wn = sqrt(K/M);

if xi < 1
    tr = ( pi - acos(xi) ) / ( wn*sqrt(1 - xi^2) );
    tp = ( pi ) / ( wn*sqrt(1 - xi^2) );
    ts = 4/xi/wn;
end

%1/(xi*wn)*log(100/90);
lambda = roots([1, 2*xi*wn, wn^2]);

s = tf('s');
H = 1/(M*s^2 + B*s + K);

figure
impulse(H)
t = (0:1e-3:2);
h = wn/sqrt(1 - xi^2).*exp(-xi*wn.*t).*sin(wn*sqrt(1-xi^2).*t)/K;
figure
plot(t,h)