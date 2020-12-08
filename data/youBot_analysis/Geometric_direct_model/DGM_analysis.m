%% youbot Geometric parameters
clear all 

addpath('../Utils/') % for matransfo
load('data_calibration_2020_11_17.mat')

dr = 0;
d2 = 0.033;
d3 = 0.302-0.147;
d4 = 0.437 - 0.302;

z0 = 0.072;
r1 = 0.147-0.072;
r5 = 0.550-0.437;
re = 0.1;

alpha = [0 pi/2 0 0 -pi/2 0];
d = [dr d2 d3 d4 0 0];
r = [z0+r1 0 0 0 r5 re];
theta = [0 pi/2 0 -pi/2 0];
theta_kuka = [169 65 -146 102.5 167.5]*pi/180; % robot offset
theta_i = theta;

th_in = [0 0 0 0 0];
th_in = thetas{1}(1,:)
%th_in = [169 65 -146 102.5 167.5]*pi/180;
theta_dh = theta_i + (th_in - theta_kuka).* [-1, -1, -1, -1, -1]; % convention ROS
theta_dh = [theta_dh 0]; % to differentiate last joint from end effector

%%

T = eye(4);
O = zeros(4,4,6);
for idx = 1:6
    tf = mattransfo(alpha(idx), d(idx), theta_dh(idx), r(idx));
    T = T * tf; 
    O(:, :, idx) = T;
end

%% Display

figure(1)
plot3(0,0,0,'r.','markersize',20); grid on; hold on
axis([-0.7 0.7 -0.7 0.7 0 +1])
plot3(O(1,4,1),O(2,4,1),O(3,4,1),'b.','markersize',20)
plot3(O(1,4,2),O(2,4,2),O(3,4,2),'b.','markersize',20)
plot3(O(1,4,3),O(2,4,3),O(3,4,3),'b.','markersize',20)
plot3(O(1,4,4),O(2,4,4),O(3,4,4),'b.','markersize',20)
plot3(O(1,4,5),O(2,4,5),O(3,4,5),'b.','markersize',20)
plot3(O(1,4,6),O(2,4,6),O(3,4,6),'g.','markersize',20)
plot3([0 O(1,4,1) O(1,4,2) O(1,4,3) O(1,4,4) O(1,4,5) O(1,4,6)], ...
    [0 O(2,4,1) O(2,4,2) O(2,4,3) O(2,4,4) O(2,4,5) O(2,4,6)], ...
    [0 O(3,4,1) O(3,4,2) O(3,4,3) O(3,4,4) O(3,4,5) O(3,4,6)],'b'); 
quiver3(O(1,4,6),O(2,4,6),O(3,4,6),O(1,1,6)/5,O(2,1,6)/5,O(3,1,6)/5,'r')
quiver3(O(1,4,6),O(2,4,6),O(3,4,6),O(1,2,6)/5,O(2,2,6)/5,O(3,2,6)/5,'g')
quiver3(O(1,4,6),O(2,4,6),O(3,4,6),O(1,3,6)/5,O(2,3,6)/5,O(3,3,6)/5,'b')
xlabel('xB'); ylabel('yB'); zlabel('zB')

%% test DGM function

th_in = [0 0 0 0 0]; % in kuka's coordinates
th_in = [169 65 -146 102.5 167.5]*pi/180;
th_in = thetas{1}(1,:);
theta_i = th_in - theta_kuka;
[t, r, ~] = DGM_youBot(theta_i, dr, re+0.1);

figure(1)
quiver3(t(1),t(2),t(3),r(1,1)/5,r(2,1)/5,r(3,1)/5,'m')
quiver3(t(1),t(2),t(3),r(1,2)/5,r(2,2)/5,r(3,2)/5,'y')
quiver3(t(1),t(2),t(3),r(1,3)/5,r(2,3)/5,r(3,3)/5,'c')