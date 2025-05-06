function [xi_dot,w_p_ref,twist_ref] = calculate_geom_progress_rate_screwbased_from_pose(T,dt,L)
% This function calculates the proposed geometric progress rate for rigid
% body trajectories from input pose data. The twist components are
% estimated from the pose data using a finite differences scheme.
% INPUT : T (4x4xN)      -> Input pose trajectory
%       : dt             -> Corresponding time step [s]
%       : L              -> value for the characteristic length
% OUTPUT: xi_dot (1xN)   -> calculated geometric progress rate 
%       : w_p_ref (3xN)  -> regulated p_perp (calculated from pose twist
%                         components, hence defined with respect to the origin of the
%                         reference frame on the object, but with coordinates expressed in
%                         the world frame).
%       : twist_ref (6xN)-> calculated regularized twist at w_p_ref, seen
%                         from the world frame.


N = size(T,3);
xi_dot = zeros(1,N-1);
p_ref = zeros(4,N-1);
omega_ref = zeros(3,N-1);
vel_ref = zeros(3,N-1);
twist = zeros(6,N-1);
for k = 1:N-1
    % calculate body-fixed twist
    twist_cross = logm_pose(inverse_T(T(:,:,k))*T(:,:,k+1))/dt;
    twist(1,k) = twist_cross(3,2);
    twist(2,k) = twist_cross(1,3);
    twist(3,k) = twist_cross(2,1);
    twist(4:6,k) = twist_cross(1:3,4);
end

for k = 1:N-1
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

w_p_ref = zeros(3,N-1);
twist_ref = [omega_ref;vel_ref];
for k = 1:N-1
    % Express quantities w.r.t. world frame
    w_p_ref(:,k) = T(1:3,4,k) + p_ref(1:3,k);
    twist_ref(1:3,k) = T(1:3,1:3,k)*twist_ref(1:3,k);
    twist_ref(4:6,k) = T(1:3,1:3,k)*twist_ref(4:6,k);
end


end