% Analytical evaluation of the 3DOF jacobian

syms th1 th2 th3 real
%syms l1 l2 l3

x0 = 0.033;
z0 = 0.147;
l1 = 0.155;
l2 = 0.1350;
l3 = 0.213;

% the reference for angles is the vertical axis (z), in the same convention
% as in the denavit hartenberg convention (see DGM_youbot), here we ignore
% the first and last joint (for the analytical part)

J = sym('J',[3,3]);

J(1,3) = l3*cos(th1+th2+th3);       % dxe/dth3
J(1,2) = l2*cos(th1+th2) + J(1,3);  % dxe/dth2
J(1,1) = l1*cos(th1) + J(1,2);      % dxe/dth1

J(2,3) = -l3*sin(th1+th2+th3);      % dze/dth3
J(2,2) = -l2*sin(th1+th2) + J(1,3); % dze/dth2
J(2,1) = -l1*sin(th1) + J(1,2);     % dze/dth1

J(3,3) = 1; % dthe/dth1
J(3,2) = 1; % dthe/dth2
J(3,1) = 1; % dthe/dth3

% Automated computation, this part takes into account 1st and last joint
% fixed rotation

[t,r,T] = DGM_youBot_sym([th1 th2 th3], 0, 0.1);
p = [t(1); t(3); acos(r(1,1))];
J2 = simplify(jacobian(p, [th1 th2 th3]));
iJ2 = vpa(simplify(inv(J2)));
dJ2 = vpa(simplify(diff(J2, th1,th2,th3)));

matlabFunction(J2,'File','J0E_3DOF');
matlabFunction(iJ2,'File','iJ0E_3DOF');
matlabFunction(dJ2,'File','dJ0E_3DOF');