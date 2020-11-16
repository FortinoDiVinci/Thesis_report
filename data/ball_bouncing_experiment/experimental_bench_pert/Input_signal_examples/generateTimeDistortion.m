function sig_dist = generateTimeDistortion(sig, t, distortion_coeff)
% The signal must have a constant sampling...
% If the distortion coefficient is less than zero, then a time compression 
% is applied, else it's a time extention

    dt = mean(diff(t));
    t2 = t(1):distortion_coeff*dt:t(end);
    t3 = t(1) + dt*(0:1:length(t2)-1);
    
    sig_tmp = interp1(t,sig,t2);
    sig_dist = interp1(t3,sig_tmp,t);
    
end

