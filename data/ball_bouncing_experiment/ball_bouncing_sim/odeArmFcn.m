function dYdt = odeArmFcn(t,Y, ma,ba,ka, t_f,f)
%%%%
% Y(1) = xa
% Y(2) = dxa

f = interp1(t_f, f, t); % time varying input force

dYdt = [ Y(2);
         (f - ba * Y(2) - ka * Y(1))/ma;];
end