function [xi_dot] = calculate_geom_progress_rate_Joris_from_twist(T,twist,L)
% This function calculates the proposed geometric progress rate for rigid
% body trajectories from input pose and twist data.
% INPUT : T (4x4xN)      -> Input pose trajectory
%       : twist (6xN)    -> Corresponding pose twist trajectory 
% OUTPUT: xi_dot (1xN)   -> calculated geometric progress rate 


N = size(T,3);
xi_dot = zeros(1,N);
for k = 1:N
    omega = twist(1:3,k);
    vel = twist(4:6,k);
    v_isa = dot(omega,vel)/norm(omega);
    xi_dot(k) = L*norm(omega) + norm(v_isa);
end

end