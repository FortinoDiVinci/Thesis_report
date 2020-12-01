function [thetas, thetas_youBot] = youBotIk3Joints(x, z, th, l1, l2, l3, x1, z1)
    %   Inverse kinematic for the youBot, when only joint 2,3and4 are actuated
    % A geometric method is used to solve the inverse kinematic

    x3 = x - l3*cos(th);
    z3 = z - l3*sin(th);

    B = z3 - z1
    A = x3 - x1

    alpha = atan2(B, A)

    C = A/cos(alpha) % length of the segment between joint 1 and joint 3

    % Al Kashi's theorem for the triangle describes by the 3 joints
    a = acos( (l1^2 + C^2 - l2^2)/(2*l1*C) )

    D = C*sin(a); % right triangle between joint 1 and 3, the 3rd point is in
    % the prolongation of the side decribed by l1

    th1 = alpha - a;
    th2 = asin(D/l2);
    th3 = th - th1 - th2

    thetas = [th1, th2, th3];

    %% in youBot angle reference
    th1_KUKA = 65*pi/180 + pi/2 - th1;
    th2_KUKA =  -146*pi/180 - th2;
    th3_KUKA =  102.5*pi/180 - th3;

    thetas_youBot = [th1_KUKA, th2_KUKA, th3_KUKA];
end

