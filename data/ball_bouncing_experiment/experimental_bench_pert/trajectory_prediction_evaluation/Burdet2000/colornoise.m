function [W,t]= colornoise(a,b,T,N)

% Function to generate colored noise using Euler-maruyama method
% For measuring,the accuracy of a numerical simulation to an SDE, the
% strong order of convergence is used.
% EM method converges with a strong order =.5
% Basically to compute the path for color noise we solve the differential
% equation : dx=axdt+bw(t) where w(t) is wiener process
% b defines the intensity of white noise
%
% Input: a,b (intensity of white noise) and T (the upper limit to time
% vector), N (the no. of samples which essentially defines the step size )
% Output: W(t)( Color noise) ,t (time vector)
% Example colornoise(1,1,1,1000)
% Author : Abhirup lahiri (abhiruplahiri@yahoo.com)

 randn('state',100);      %fixing the state of generator
 dt=T/N;
 dW=sqrt(dt)*randn(1,N);

 R=4;                      %fixed for this computation
 L=N/R;
 Xem=zeros(1,L);
 Xzero=0;
 Xtemp=Xzero;
 Dt=R*dt;

 for j=1:L
    Winc=sum(dW(R*(j-1)+1:R*j));
    Xtemp=Xtemp+a*Dt*Xtemp+b*Winc;
    Xem(j)=Xtemp;
 end

 W=[Xzero,Xem];
 t=[0:Dt:T];