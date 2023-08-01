function [c,ceq] = calculate_nonlinear_inequality_constraint(L,posetwist,dt,params)
% This function sets the nonlinear inequality constraint regarding the
% calibration procedure for L. (at least alpha percent of the trajectory has to be
% represented as a translational motion)

N = size(posetwist,2);

s_ref = 0;
s = 0;
for k = 1:N
    omega = posetwist(1:3,k);
    vel = posetwist(4:6,k);
    omega_ref(:,k) = omega;
    if norm(omega) == 0
        vel_ref(:,k) = vel;
        v1 = norm(vel_ref(:,k));
    else
        p_perp = cross(omega,vel)/dot(omega,omega);
        if norm(p_perp) > L
            p_boundary = L*p_perp/norm(p_perp);
            vel_ref(:,k) = vel + cross(omega,p_boundary);
            v1 = norm(vel_ref(:,k));
        else
            v1 = dot(vel,omega)/norm(omega);
            vel_ref(:,k) = v1*omega/norm(omega);
        end
    end
    s_ref = s_ref + norm(vel)*dt;
    s = s + abs(v1)*dt;
end
c = abs(s-s_ref)/s_ref + params.alpha-1; 
ceq = [];
end