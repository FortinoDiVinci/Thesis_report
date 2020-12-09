function [ddth, dth, th] = DynModel_3DOF(f_env,tau,th,dth,dt)
% Dynamic equation of the KUKA robot
% call it as: [thetapp] = DynModel_3DOF(f_env,tau,q,dq)
% where:
%        - M,C,G are the inertia, Coriolis and gravity matrices
%        - tau is a vector of joint torques
%        - q, dq, ddq joint angles, velocities and accelerations

    th2 = th(1);
    th3 = th(2);
    th4 = th(3);
    
    dth2 = dth(1);
    dth3 = dth(2);
    dth4 = dth(3);
    
    ddth = iM_3DOF(th3,th4)*(tau - C_3DOF(dth2,dth3,dth4, th3,th4)*dth - G_3DOF(th2,th3,th4) - J0E_3DOF(th2,th3,th4)'*f_env);
    if(isnan(ddth))
        error('ddth is NaN...')
    end
    % integration
    dth = ddth .*dt + dth;
    th = dth .*dt + th;

end
