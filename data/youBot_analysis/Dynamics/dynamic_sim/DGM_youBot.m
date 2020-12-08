function [t, r, T, joint_pos] = DGM_youBot(th, x0, le)
% Direct geometric model of the youBot robot
% The input angles are expected to be in the DH convention
% If the angle from KUKA youBot are introduced, the outcome
% will be wrong

% theta_i = theta_kuka - offsets
% offsets = [169 65 -146 102.5 167.5]*pi/180
% theta_i should be a horizontal vector of size 5

    % geometric parameters are fixed
    d2 = 0.033;
    d3 = 0.302-0.147;
    d4 = 0.437 - 0.302;

    r1 = 0.147;
    r5 = 0.550-0.437;
    
    th0 = 0;
    th5 = 0;
    ths = -110*pi/180;

    %          TH1   TH2    TH3    TH4    TH5   EE
    alpha    = [0    pi/2    0      0    -pi/2  0 ]';
    d        = [x0    d2     d3     d4     0    0 ]';
    r        = [r1    0      0      0      r5   le]';
    theta    = [th0  th(1)  th(2)  th(3)  th5  ths]';
    
    T = eye(4);
    joint_pos = zeros(3,6);
    for idx = 1:6
        tf = mattransfo(alpha(idx), d(idx), theta(idx), r(idx));
        T = T * tf; 
        joint_pos(:,idx) = T(1:3,4);
    end
    
    t = T(1:3,4); % translation
    r = T(1:3,1:3); % rotation
    
end

function res = mattransfo(alpha, d, theta, r)

res = [cos(theta) -sin(theta) 0 d; ...
       cos(alpha)*sin(theta) cos(alpha)*cos(theta) -sin(alpha) -r*sin(alpha);...
       sin(alpha)*sin(theta) sin(alpha)*cos(theta) cos(alpha) r*cos(alpha); ...
       0 0 0 1];
end
