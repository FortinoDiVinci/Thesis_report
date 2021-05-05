function tau_fs = Fs_3DOF(dq, fe, Fs, dv)
%FS_3DOF Summary of this function goes here
%   Detailed explanation goes here

    if nargin < 4
        dv = 1e-3;
    end
    if nargin < 3
        %Fs = [0.97571; 0.65131; 0.25819];
        Fs = [0.9;  1.3; 0.5];
    end
   
    Fc = Fs;
    
    for ii = 1:length(dq)
        if abs(dq(ii)) < dv
            tau_fs(ii,1) = sign(fe(ii)).*min(abs(fe(ii)), Fs(ii));
        else
            tau_fs(ii,1) = sign(dq(ii)).*Fc(ii);
        end
    end
    
    
end

