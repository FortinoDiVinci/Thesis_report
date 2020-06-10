function dYdt = odeBallisticFcn(t,Y,g)
%%%%
% Y(1) = xb
% Y(2) = dxb

dYdt = [ Y(2);
         -g];
end
