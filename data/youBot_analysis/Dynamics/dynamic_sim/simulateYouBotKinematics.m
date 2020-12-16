function simulateYouBotKinematics(q,dt,f,p0,pert)
% This function display the youbot joint behaviour 

%     q1 = q(:,1);
%     q2 = q(:,2);
%     q3 = q(:,3);
%     q4 = q(:,4);
%     q2 = q(:,5);

%     fx = f(:,1);
%     fy = f(:,2);
%     fz = f(:,3);

    if size(q, 2) ~= 5
        error('Wrong dimensions, or number of joints.')
    end
    if size(q, 1) > 1
        % downsampling
        if dt < 2e-2 % 50Hz
            dwnsamp = floor(2e-2/dt);
        end
        sq = q(1:dwnsamp:end,:);
        if nargin > 4
            spert = pert(1:dwnsamp:end,:);
        end
        if nargin > 3
            sp0 = p0(1:dwnsamp:end,:);
        end
        if nargin > 2
            sf = f(1:dwnsamp:end,:);
        end
    else
        sq = q;
        if nargin > 4
            spert = pert;
        end
        if nargin > 3
            sp0 = p0;
        end
        if nargin > 2
            sf = f;
        end
    end
    
    
    
    % Numeric Values of the robot's geometrical parameters
    r1 = 0.147;
    d2 = 0.033;
    d3 = 0.155;
    d4 = 0.135;
    r5 = 0.113; 
    rs = 0.1;

    %          TH1   TH2  TH3  TH4  TH5   EE
    alpha    = [0   pi/2  0    0  -pi/2  0 ];
    d        = [0    d2   d3   d4   0    0 ];
    r        = [r1   0    0    0    r5   rs];
    %theta_dh = [0   pi/2  0  -pi/2  0    0 ];

    sq = [sq, zeros(size(sq(:,1)))];
    theta = sq;
    joint_pos = zeros(3,size(theta,2),size(theta,1));
    
    for k = 1:size(theta,1)
        T = eye(4);
        for j = 1:size(theta,2)
            tf = mattransfo(alpha(j),d(j),theta(k,j),r(j));
            T = T * tf;
            joint_pos(:,j,k) = T(1:3,4);
        end
    end
    
    figure
    for k = 1:size(sq,1)
        O1 = joint_pos(:,1,k);
        O2 = joint_pos(:,2,k);
        O3 = joint_pos(:,3,k);
        O4 = joint_pos(:,4,k);
        O5 = joint_pos(:,5,k);
        OE = joint_pos(:,6,k);
        hold off
        plot3(0,0,0,'bs','markersize',10)
        hold on, grid on
        plot3(O1(1),O1(2),O1(3),'b.','markersize',20)
        view(5,0)
        plot3(O2(1),O2(2),O2(3),'b.','markersize',20)
        plot3(O3(1),O3(2),O3(3),'b.','markersize',20)
        plot3(O4(1),O4(2),O4(3),'b.','markersize',20)
        plot3(O5(1),O5(2),O5(3),'b.','markersize',20)
        plot3(OE(1),OE(2),OE(3),'g.','markersize',20) % endpoint
        % segments
        plot3([0 O1(1) O2(1) O3(1) O4(1) O5(1) OE(1)],...
            [0 O1(2) O2(2) O3(2) O4(2) O5(2) OE(2)],...
            [0 O1(3) O2(3) O3(3) O4(3) O5(3) OE(3)],'b'); 
        % Perturbation introduced
        if nargin > 4
            if any(spert(k,:)) ~= 0
                plot3(OE(1),OE(2),OE(3),'r.','markersize',30)
            end
        end
        % Target or point of interest
        if nargin > 3
            plot3(sp0(k,1),sp0(k,2),sp0(k,3),'mp','markersize',13)
        end
        % z force
        if nargin > 2
            quiver3(OE(1),OE(2),OE(3),sf(k,1)/10,sf(k,2)/10,sf(k,3)/10,'r')  
        end
        axis([-0.3 0.3 -0.3 0.3 0 0.5])
        pause(0.02) % 1kHz
    end
    
end

function res = mattransfo(alpha, d, theta, r)

res = [cos(theta) -sin(theta) 0 d; ...
       cos(alpha)*sin(theta) cos(alpha)*cos(theta) -sin(alpha) -r*sin(alpha);...
       sin(alpha)*sin(theta) sin(alpha)*cos(theta) cos(alpha) r*cos(alpha); ...
       0 0 0 1];
end