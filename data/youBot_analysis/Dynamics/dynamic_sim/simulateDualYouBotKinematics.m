function simulateDualYouBotKinematics(tha,thb,dt,f,p0,pert)
% This function display the youbot joint behaviour 

%     th1 = tha(:,1);
%     th2 = tha(:,2);
%     th3 = tha(:,3);
%     th4 = tha(:,4);
%     th2 = tha(:,5);

%     fx = f(:,1);
%     fy = f(:,2);
%     fz = f(:,3);

    if size(tha) ~= size(thb)
        error('The multi dimentional arrays th1 and th2 must have the same size')
    end

    if size(tha, 2) ~= 5
        error('Wrong dimensions, or number of joints.')
    end
    
    if size(tha, 1) > 1
        % downsampling
        if dt < 2e-2 % 50Hz
            dwnsamp = floor(2e-2/dt);
        end
        stha = tha(1:dwnsamp:end,:);
        sthb = thb(1:dwnsamp:end,:);
        if nargin > 5
            spert = pert(1:dwnsamp:end,:);
        end
        if nargin > 4
            sp0 = p0(1:dwnsamp:end,:);
        end
        if nargin > 3
            sf = f(1:dwnsamp:end,:);
        end
    else
        stha = tha;
        sthb = thb;
        if nargin > 5
            spert = pert;
        end
        if nargin > 4
            sp0 = p0;
        end
        if nargin > 3
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

    stha = [stha, zeros(size(stha(:,1)))];
    sthb = [sthb, zeros(size(sthb(:,1)))];
    
    joint_posA = zeros(3,size(stha,2),size(stha,1));
    joint_posB = zeros(3,size(sthb,2),size(sthb,1));
    
    for k = 1:size(stha,1)
        Ta = eye(4);
        Tb = eye(4);
        for j = 1:size(stha,2)
            tf = mattransfo(alpha(j),d(j),stha(k,j),r(j));
            Ta = Ta * tf;
            joint_posA(:,j,k) = Ta(1:3,4);
            %
            tf = mattransfo(alpha(j),d(j),sthb(k,j),r(j));
            Tb = Tb * tf;
            joint_posB(:,j,k) = Tb(1:3,4);
        end
    end
    
    figure
    for k = 1:size(stha,1)
        O1a = joint_posA(:,1,k);
        O2a = joint_posA(:,2,k);
        O3a = joint_posA(:,3,k);
        O4a = joint_posA(:,4,k);
        O5a = joint_posA(:,5,k);
        OEa = joint_posA(:,6,k);
        O1b = joint_posB(:,1,k);
        O2b = joint_posB(:,2,k);
        O3b = joint_posB(:,3,k);
        O4b = joint_posB(:,4,k);
        O5b = joint_posB(:,5,k);
        OEb = joint_posB(:,6,k);
        hold off
        % A
        plot3(0,0,0,'bs','markersize',10)
        hold on, grid on
        plot3(O1a(1),O1a(2),O1a(3),'b.','markersize',20)
        view(5,0)
        plot3(O2a(1),O2a(2),O2a(3),'b.','markersize',20)
        plot3(O3a(1),O3a(2),O3a(3),'b.','markersize',20)
        plot3(O4a(1),O4a(2),O4a(3),'b.','markersize',20)
        plot3(O5a(1),O5a(2),O5a(3),'b.','markersize',20)
        plot3(OEa(1),OEa(2),OEa(3),'g.','markersize',20) % endpoint
        % segments
        plot3([0 O1a(1) O2a(1) O3a(1) O4a(1) O5a(1) OEa(1)],...
            [0 O1a(2) O2a(2) O3a(2) O4a(2) O5a(2) OEa(2)],...
            [0 O1a(3) O2a(3) O3a(3) O4a(3) O5a(3) OEa(3)],'b'); 
        % B
        plot3(O1b(1),O1b(2),O1b(3),'c.','markersize',20)
        plot3(O2b(1),O2b(2),O2b(3),'c.','markersize',20)
        plot3(O3b(1),O3b(2),O3b(3),'c.','markersize',20)
        plot3(O4b(1),O4b(2),O4b(3),'c.','markersize',20)
        plot3(O5b(1),O5b(2),O5b(3),'c.','markersize',20)
        plot3(OEb(1),OEb(2),OEb(3),'g.','markersize',20) % endpoint
        % segments
        plot3([0 O1b(1) O2b(1) O3b(1) O4b(1) O5b(1) OEb(1)],...
            [0 O1b(2) O2b(2) O3b(2) O4b(2) O5b(2) OEb(2)],...
            [0 O1b(3) O2b(3) O3b(3) O4b(3) O5b(3) OEb(3)],'c');
        % Perturbation introduced
        if nargin > 5
            if any(spert(k,:)) ~= 0
                plot3(OEa(1),OEa(2),OEa(3),'r.','markersize',30)
                plot3(OEb(1),OEb(2),OEb(3),'r.','markersize',30)
            end
        end
        % Target or point of interest
        if nargin > 4
            plot3(sp0(k,1),sp0(k,2),sp0(k,3),'mp','markersize',13)
        end
        % z force
        if nargin > 3
            quiver3(OEa(1),OEa(2),OEa(3),sf(k,1)/10,sf(k,2)/10,sf(k,3)/10,'r')  
            quiver3(OEb(1),OEb(2),OEb(3),sf(k,1)/10,sf(k,2)/10,sf(k,3)/10,'r') 
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