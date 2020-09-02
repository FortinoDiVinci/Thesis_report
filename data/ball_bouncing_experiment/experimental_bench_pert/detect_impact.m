function [j_counter] = detect_impact(vz_b, idx, window)
%DETECT_IMPACT Summary of this function goes here
%   Detailed explanation goes here

    for i = idx + 1:length(vz_b)
        
        if (i - window) <= 0
            continue
        end
        
        if ( mean(vz_b(i - window:i-1)) < vz_b(i) )
            j_counter = i;
            break
        end
    end

    if( ~exist('j_counter'))
        j_counter = 0;
    end
    
end

