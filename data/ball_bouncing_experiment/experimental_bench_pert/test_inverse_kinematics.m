
addpath('../../youBot_analysis/Utils')

DISPLAY_ANIMATION = 0;
DISPLAY_ANIMATION_2 = 1;

x1 = 0.033;
z1 = 0.147;

l1 = 0.302 - 0.147;
l2 = 0.437 - 0.302;
l3 = 0.550 - 0.437 + 0.068;

z = (0.147+0.155)-0.028 + 0.02*sin([0:0.04:4*pi]);
x = -(0.135+0.2175)*ones(size(z));%-0.125*ones(size(z));%
th = pi;

thetas_rob = NaN(length(z), 3);

for ii = 1:length(z)
    [thetas(ii,:), thetas_rob(ii,:)] = youBotIk3Joints(x(ii), z(ii), th, l1, l2, l3, x1, z1);
end

x2 = x1 + l1*cos(thetas(:,1));
z2 = z1 + l1*sin(thetas(:,1));

x3 = x2 + l2*cos(thetas(:,1)+thetas(:,2));
z3 = z2 + l2*sin(thetas(:,1)+thetas(:,2));

xe = x3 + l3*cos(thetas(:,1)+thetas(:,2)+thetas(:,3));
ze = z3 + l3*sin(thetas(:,1)+thetas(:,2)+thetas(:,3));

if DISPLAY_ANIMATION 
    for ii = 1:length(z)
        figure(2)
        plot(0,0,'r.','markersize',20); grid on; hold on
        plot(x1,z1, 'b.','markersize',20);
        plot(x2(ii),z2(ii), 'b.','markersize',20);
        plot(x3(ii),z3(ii), 'b.','markersize',20);
        plot(xe(ii),ze(ii), 'g.','markersize',20);
        line([0, x1], [0, z1], 'Color','black','linewidth',2);
        line([x1, x2(ii)], [z1, z2(ii)], 'Color','black','linewidth',2);
        line([x2(ii) x3(ii)], [z2(ii), z3(ii)], 'Color','black','linewidth',2);
        line([x3(ii) xe(ii)], [z3(ii), ze(ii)], 'Color','black','linewidth',2);
        hold off
        axis([-0.4 0.4 0 0.6]);
        pause(.1)
    end
end

if DISPLAY_ANIMATION_2 
    for ii = 1:length(z)
        T02 = MGD_T02(169*pi/180,thetas_rob(ii,1));
        T03 = MGD_T03(169*pi/180,thetas_rob(ii,1),thetas_rob(ii,2));
        T04 = MGD_T04(169*pi/180,thetas_rob(ii,1),thetas_rob(ii,2),thetas_rob(ii,3));
        T0end = MGD_T0handle(169*pi/180,thetas_rob(ii,1),thetas_rob(ii,2),thetas_rob(ii,3),0);
        figure(2)
        subplot(2,1,1)
        plot(0,0,'r.','markersize',20); grid on; hold on
        plot(T02(1,4), T02(3,4), 'b.','markersize',20);
        plot(T03(1,4), T03(3,4), 'b.','markersize',20);
        plot(T04(1,4), T04(3,4), 'b.','markersize',20);
        plot(T0end(1,4), T0end(3,4), 'g.','markersize',20);
        line([0, T02(1,4)], [0, T02(3,4)], 'Color','black','linewidth',2);
        line([T02(1,4), T03(1,4)], [T02(3,4), T03(3,4)], 'Color','black','linewidth',2);
        line([T03(1,4), T04(1,4)], [T03(3,4), T04(3,4)], 'Color','black','linewidth',2);
        line([T04(1,4), T0end(1,4)], [T04(3,4), T0end(3,4)], 'Color','black','linewidth',2);
        plot(x(ii), z(ii), 'r*','markersize',20)
        hold off        
        axis([-0.4 0.4 0 0.6]);
        subplot(2,1,2)
        
        plot(0,0,'r.','markersize',20); grid on; hold on
        plot(x1,z1, 'b.','markersize',20);
        plot(x2(ii),z2(ii), 'b.','markersize',20);
        plot(x3(ii),z3(ii), 'b.','markersize',20);
        plot(xe(ii),ze(ii), 'g.','markersize',20);
        line([0, x1], [0, z1], 'Color','black','linewidth',2);
        line([x1, x2(ii)], [z1, z2(ii)], 'Color','black','linewidth',2);
        line([x2(ii) x3(ii)], [z2(ii), z3(ii)], 'Color','black','linewidth',2);
        line([x3(ii) xe(ii)], [z3(ii), ze(ii)], 'Color','black','linewidth',2);
        plot(x(ii), z(ii), 'r*','markersize',20)
        hold off
        axis([-0.4 0.4 0 0.6]);
        pause(.01)
    end
end


T02 = MGD_T02(169*pi/180,2.76);
T03 = MGD_T03(169*pi/180,2.76,-4.02);
T04 = MGD_T04(169*pi/180,2.76,-4.02,2.61);
T0end = MGD_T0handle(169*pi/180,2.76,-4.02,2.61,0);
figure(2)
plot(0,0,'r.','markersize',20); grid on; hold on
plot(T02(1,4), T02(3,4), 'b.','markersize',20);
plot(T03(1,4), T03(3,4), 'b.','markersize',20);
plot(T04(1,4), T04(3,4), 'b.','markersize',20);
plot(T0end(1,4), T0end(3,4), 'g.','markersize',20);
line([0, T02(1,4)], [0, T02(3,4)], 'Color','black','linewidth',2);
line([T02(1,4), T03(1,4)], [T02(3,4), T03(3,4)], 'Color','black','linewidth',2);
line([T03(1,4), T04(1,4)], [T03(3,4), T04(3,4)], 'Color','black','linewidth',2);
line([T04(1,4), T0end(1,4)], [T04(3,4), T0end(3,4)], 'Color','black','linewidth',2);
plot(x(ii), z(ii), 'r*','markersize',20)
axis([-0.4 0.4 0 0.6]);

%%
% 
% th1_KUKA = 0*pi/180;
% th2_KUKA = 0*pi/180;
% th3_KUKA = 0*pi/180;
% 
% % theta's offsets
% th1os = 65*pi/180;
% th2os =  -146*pi/180;
% th3os =  102.5*pi/180;
% 
% th1 = th1os + pi/2 - th1_KUKA;
% th2 = th2os - th2_KUKA;
% th3 = th3os - th3_KUKA;
% 
% O1 = [x1, z1];
% O2 = [x1+l1*cos(th1), z1+l1*sin(th1)];
% O3 = [x1+l1*cos(th1)+l2*cos(th1+th2), z1+l1*sin(th1)+l2*sin(th1+th2)];
% Oe = [x1+l1*cos(th1)+l2*cos(th1+th2)+l3*cos(th1+th2+th3),z1+l1*sin(th1)+l2*sin(th1+th2)+l3*sin(th1+th2+th3)];
% 
% figure(2)
% plot(0,0,'r.','markersize',20); grid on; hold on
% plot(x1,z1, 'b.','markersize',20);
% plot(O2(1), O2(2), 'b.','markersize',20);
% plot(O3(1), O3(2), 'b.','markersize',20);
% plot(Oe(1), Oe(2), 'g.','markersize',20);
% line([0, x1], [0, z1], 'Color','black','linewidth',2);
% line([x1, O2(1)], [z1, O2(2)], 'Color','black','linewidth',2);
% line([O2(1) O3(1)], [O2(2), O3(2)], 'Color','black','linewidth',2);
% line([O3(1) Oe(1)], [O3(2), Oe(2)], 'Color','black','linewidth',2);
% hold off
% axis([-0.2 0.5 0 0.7]);

