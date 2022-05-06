function Q_red = orderRed(Q,n_red)

[Q_bal,Gq] = balreal(Q);
n = length(Q.a);
elimQ = (Gq<Gq(n_red));          % small entries of g -> negligible states
%elimQ = [zeros(n_red,1);ones(n-n_red,1)];
Q_red = modred(Q_bal,elimQ);  