function simulateYouBot_3DOF(q,f)
% This function display the youbot joint behaviour only considering the 
% joints 2,3 and 4.

%     q2 = q(1,:);
%     q3 = q(2,:);
%     q4 = q(3,:);
    
    % Numeric Values of the robot's geometrical parameters
    d1 = 0.024;
    r1 = 0.147;
    d2 = 0.033;
    d3 = 0.155;
    d4 = 0.135;
    d5 = 0.218; % length of the end effector
    d = [d2 d3 d4 d5]';
    alpha1 = pi/2;
    alpha2 = 0;
    alpha3 = 0;
    alpha = [alpha1 alpha2 alpha3 0]';
    r = [r1 0 0 0]';
    
    q = [q; zeros(1,size(q,2))];
    for k = 1:size(q,2)
        g0E = ones(4,4);
        for j = 1:4
            g0E = g0E * mattransfo(alpha(j),d(j),q(j,k),r(j));
            joint_pos(:,j,k) = g0E(1:3,4);
        end
    end
    
    figure
    for k = 1:size(q,2)
        O1 = joint_pos(:,1,k);
        O2 = joint_pos(:,2,k);
        O3 = joint_pos(:,3,k);
        OE = joint_pos(:,4,k);
        hold off
        plot3(O1(1),O1(2),O1(3),'b.','markersize',20)
        view(90,0)
        hold on, grid on
        plot3(O2(1),O2(2),O2(3),'b.','markersize',20)
        plot3(O3(1),O3(2),O3(3),'b.','markersize',20)
        plot3(OE(1),OE(2),OE(3),'g.','markersize',20) % endpoint
        % segments
        plot3([0 O1(1) O2(1) O3(1) OE(1)],...
            [0 O1(2) O2(2) O3(2) OE(2)],...
            [0 O1(3) O2(3) O3(3) OE(3)],'b'); 
        % z force
        quiver3(OE(1),OE(2),OE(3),f(1,k),f(2,k),f(3,k),'r')   
        pause(0.001) % 1kHz
    end
    
end

