function dYdt = odeImpactFcn(t,Y, kb,ka,mb,ma,bb,ba,r,g,xv, t_f,f)
%%%%
% Y(1) = xb
% Y(2) = xa
% Y(3) = dxb
% Y(4) = dxa

f = interp1(t_f, f, t); % time varying input force

dYdt = [ Y(3);
         Y(4);
         -kb/mb*(Y(1) - Y(2) - r) - bb/mb*Y(3) - g;
         -ka/ma*(Y(2) - xv) + kb/ma*(Y(1) - Y(2) - r) - ba/ma*Y(4) + bb/mb*Y(3) + f];
end
