function [xi_dot,w_p_ref,twist_ref] = calculate_geom_progress_rate_screwbased_from_twist(T,twist,L)
% This function calculates the proposed geometric progress rate for rigid
% body trajectories from input pose and twist data.
% INPUT : T (4x4xN)      -> Input pose trajectory
%       : twist (6xN)    -> Corresponding pose twist trajectory 
%       : L              -> value for the characteristic length
% OUTPUT: xi_dot (1xN)   -> calculated geometric progress rate 
%       : w_p_ref (3xN)  -> regulated p_perp (calculated from pose twist
%                         components, hence defined with respect to the origin of the
%                         reference frame on the object, but with coordinates expressed in
%                         the world frame).
%       : twist_ref (6xN)-> calculated regularized twist at w_p_ref, seen
%                         from the world frame.

N = size(T,3);
xi_dot = zeros(1,N);
p_ref = zeros(4,N);
omega_ref = zeros(3,N);
vel_ref = zeros(3,N);
for k = 1:N
    omega = twist(1:3,k);
    vel = twist(4:6,k);
    omega_ref(:,k) = omega;
    if norm(omega) == 0
        vel_ref(:,k) = vel;
        v1 = norm(vel_ref(:,k));
        p_ref(:,k) = [0;0;0;1];
    else
        p_perp = cross(omega,vel)/dot(omega,omega);
        if norm(p_perp) > L
            p_boundary = L*p_perp/norm(p_perp);
            vel_ref(:,k) = vel + cross(omega,p_boundary);
            v1 = norm(vel_ref(:,k));
            p_ref(:,k) = [p_boundary;1];
        else
            v1 = dot(vel,omega)/norm(omega);
            vel_ref(:,k) = v1*omega/norm(omega);
            p_ref(:,k) = [p_perp;1];
        end
    end
    xi_dot(k) = sqrt(L^2*dot(omega,omega) + v1^2);
end

w_p_ref = zeros(3,N);
twist_ref = [omega_ref;vel_ref];
for k = 1:N
    % Express quantities w.r.t. world frame
    w_p_ref(:,k) = T(1:3,4,k) + p_ref(1:3,k);
    twist_ref(1:3,k) = T(1:3,1:3,k)*twist_ref(1:3,k);
    twist_ref(4:6,k) = T(1:3,1:3,k)*twist_ref(4:6,k);
end

end