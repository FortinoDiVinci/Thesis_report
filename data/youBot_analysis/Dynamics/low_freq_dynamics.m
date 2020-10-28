%% Symbolic

J = sym('J%d%d', [3 3]);
M = sym('M%d%d', [3 3]);
K = sym('K%d%d', [3 3]);

syms kvi ki

% M is symetric
M(1,2) = M(2,1);
M(1,3) = M(3,1);
M(2,3) = M(3,2);

Kvi = sym(eye(3,3))*kvi;
Ki = sym(zeros(3,3));
Ki(2,2) = ki;

iJ = inv(J);
iM = inv(M);

A = simplify(J*iM*Kvi*iJ*Ki);
B = simplify(J*(iM*Kvi*iJ - K*iJ));

%%

H_low_freq = inv(B)*A;

simplify(H_low_freq(2,2))

save('h_low_freq.mat', 'H_low_freq');