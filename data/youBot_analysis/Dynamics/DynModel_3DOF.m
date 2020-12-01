function [ddq, dq, q] = DynModel_3DOF(f_env,tau,q,dq,dt)
% Dynamic equation of the KUKA robot
% call it as: [thetapp] = DynModel_3DOF(f_env,tau,q,dq)
% where:
%        - M,C,G are the inertia, Coriolis and gravity matrices
%        - tau is a vector of joint torques
%        - q, dq, ddq joint angles, velocities and accelerations

    ddq = M_3DOF(q(2:3)) \ ...
        (tau - C_3DOF(q(2:3),dq(1:3))*dq - G_3DOF(q(1:3)) - J0E_3DOF(q(1:3))'*f_env);
    if(isnan(ddq))
        error('ddq is NaN...')
    end
    % integration
    dq = ddq .*dt + dq;
    q = dq .*dt + q;

end
