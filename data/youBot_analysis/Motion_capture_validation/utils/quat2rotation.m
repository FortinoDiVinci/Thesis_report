function [R] = quat2rotation(q)
%QUAT2ROTATION gives the rotation matrix from a quaternion 
%   q = [qw, qx, qy, qz]
    
    if (length(q) ~= 4)
        error('The function expect a quaternion as input, q = [qw, qx, qy, qz]');
    end
    
    a = q(1);
    b = q(2);
    c = q(3);
    d = q(4);
    z = sqrt(a^2 + b^2 + c^2 + d^2);
    
    if abs(1 - z) > 1e-5
        error('The quaternion given is not unitary ! z = %d', z);     
    end

    R11 = a^2 + b^2 - c^2 - d^2;
    R12 = 2*a*d + 2*b*c;
    R13 = 2*b*d - 2*a*c;
    
    R21 = 2*b*c - 2*a*d;
    R22 = a^2 - b^2 + c^2 - d^2;
    R23 = 2*a*b + 2*c*d;
    
    R31 = 2*a*c + 2*b*d;
    R32 = 2*c*d - 2*a*b;
    R33 = a^2 - b^2 - c^2 + d^2;
    
    R = [R11, R21, R31;
         R12, R22, R32;
         R13, R23, R33];  
end

