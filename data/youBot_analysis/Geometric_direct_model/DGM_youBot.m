function [t, r, T] = DGM_youBot(theta_i, x0, le)
% Direct geometric model of the youBot robot
% The input angles are expected to be in the DH convention
% If the angle from KUKA youBot are introduced, the outcome
% will be wrong

% theta_i = theta_kuka - offsets
% offsets = [169 65 -146 102.5 167.5]*pi/180
% theta_i should be a horizontal vector of size 5

    if all(size(theta_i) == [5,1])
        theta_i = theta_i';
    elseif ~all(size(theta_i) == [1,5])
        error("A vector of 5 elements was expected as first argument." + ...
            " The dimension of the first argument are: " + num2str(size(theta_i)))
    end

    % geometric parameters are fixed
    d2 = 0.033;
    d3 = 0.302-0.147;
    d4 = 0.437 - 0.302;

    z0 = 0.072;
    r1 = 0.147-0.072;
    r5 = 0.550-0.437;

    %          TH1   TH2  TH3  TH4  TH5   EE
    alpha    = [0    pi/2  0    0  -pi/2  0 ];
    d        = [x0    d2   d3   d4   0    0 ];
    r        = [z0+r1 0    0    0    r5   le];
    theta_dh = [0    pi/2  0  -pi/2  0    0 ];

    theta = theta_dh - [theta_i, 0];
    
    T = eye(4);
    
    for idx = 1:6
        tf = mattransfo(alpha(idx), d(idx), theta(idx), r(idx));
        T = T * tf; 
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
