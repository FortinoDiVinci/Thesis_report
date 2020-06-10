%%%% contains errors...

%% Symbolic differential solving

syms xa(t) xb(t) 

syms ma mb ka kb r xv g positive
syms vb0 h0

assumeAlso(ma, 'real')
assumeAlso(mb, 'real')
assumeAlso(ka, 'real')
assumeAlso(kb, 'real')
assumeAlso(r, 'real')
assumeAlso(xv, 'real')
assumeAlso(g, 'real')

% decoupled differential equations
ode1 = diff(xa,4)*ma*mb + diff(xa,2)*(ka*mb + kb*(mb + ma)) + xa*ka*kb + mb*kb*g - ka*xv == 0;
ode2 = diff(xb,4)*ma*mb + diff(xb,2)*(ka*mb + kb*(mb + ma)) + xb*ka*kb + mb*g*(ka + kb) - kb*(r + xv) == 0;

dxa = diff(xa);
d2xa = diff(dxa);
d3xa = diff(d2xa);
dxb = diff(xb);
d2xb = diff(dxb);
d3xb = diff(d2xb);

cond1a = xa(0) == 0;
cond2a = dxa(0) == 0;
cond3a = d2xa(0) == 0;
cond4a = d3xa(0) == 0;
cond1b = xb(0) == h0;
cond2b = dxb(0) == vb0;
cond3b = d2xb(0) == g;
cond4b = d3xb(0) == 0;

conds1 = [cond1a; cond2a; cond3a; cond4a]; 
conds2 = [cond1b; cond2b; cond3b; cond4b];

s_xa = dsolve(ode1, conds1);
s_xb = dsolve(ode2, conds2);

s_xa = vpa(simplify(s_xa), 5)
s_xb = vpa(simplify(s_xb), 5)

% coupled differential equations

ode3 = mb*diff(xb,2) + kb*(xb - xa - r) + mb*g == 0;
ode4 = ma*diff(xa,2) + ka*(xa - xv) - kb*(xb - xa - r) == 0;

conds = [cond1a; cond2a; cond3a; cond1b; cond2b; cond3b];
odes = [ode3; ode4];

[s_xa2, s_xb2] = dsolve(odes, conds);


%% Characteristic polynomial 

syms x

polyn1 = x^4*ma*mb + x^2*(ka*mb + kb*(mb + ma)) + x*ka*kb + mb*kb*g - ka*xv == 0;
root(polyn1,x)
s_polyn1 = simplify(solve(polyn1, x, 'MaxDegree',4));


%% assume typical value

ode1v = subs(ode1, [ma mb ka kb r xv g], [0.113, 0.025, 500, 1500, 0.05, 0, 9.81])
ode2v = subs(ode2, [ma mb ka kb r xv g], [0.113, 0.025, 500, 1500, 0.05, 0, 9.81])

conds1v = subs(conds1, [ma mb ka kb r xv g vb0 h0], [0.113, 0.025, 500, 1500, 0.05, 0, 9.81, -3, 0.05])
conds2v = subs(conds2, [ma mb ka kb r xv g vb0 h0], [0.113, 0.025, 500, 1500, 0.05, 0, 9.81, -3, 0.05])

s_xa = dsolve(ode1v,conds1v)
s_xb = dsolve(ode2v,conds2v)

%%%
odesv = subs(odes, [ma mb ka kb r xv g], [0.113, 0.025, 500, 1500, 0.05, 0, 9.81])
condsv = subs(conds, [ma mb ka kb r xv g vb0 h0], [0.113, 0.025, 500, 1500, 0.05, 0, 9.81, -3, 0.05])

[s2_xa, s2_xb] = dsolve(odesv, conds)

%% numerical ode solving

Kb = 1500;      % 1500 N/m
Ka = 500;       % 500 N/m
Mb = 0.025;     % 25 g
Ma = 1;         % 1 kg
R = 0.05;       % 5 cm
G = 9.81;
Xv = 0;

t_interval = [0,0.02];
init_cond = [R,0,-3,0]'; % [xb,xa,dxb,dxa]

[t,y] = ode45(@(t,Y) odeImpactFcn(t,Y, Kb,Ka,Mb,Ma,R,G,Xv) , t_interval , init_cond);
figure
subplot(2,1,1)
plot(t,y(:,1),'b',t,y(:,2),'r');
subplot(2,1,2)
plot(t,y(:,1)-y(:,2))

vb_post_impact = (y(25,1) - y(23,1))/ (t(25) - t(23));