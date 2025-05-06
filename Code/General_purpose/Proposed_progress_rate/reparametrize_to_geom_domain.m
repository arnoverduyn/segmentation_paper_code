function [T_s,geom_progress,s_t] = reparametrize_to_geom_domain(T,dt,s_dot,ds)
% This function reparametrizes the input temporal pose trajectory to a
% geometric domain, using the input geometric progress rate. The
% reparameterization is performed by linear interpolation.
% INPUT: T (4x4xN)    -> Input temporal pose trajectory
%      : dt           -> time step [s]
%      : s_dot (1xN)  -> Corresponding geometric progress rate
%      : ds           -> desired equidistant progress step [m]
% OUTPUT: T_s (4x4xN) -> Reparametrized geometric pose trajectory
%       : s (1xN)     -> Corresponding geometric progress vector 

N = size(T,3);
s = [0];
for k = 1:N-1
    s(k+1) = s(k) + s_dot(k)*dt;
end
s_t = s;

% equidistant s
geom_progress = 0:ds:s(end);
T_s = interpT(s,T,geom_progress);

end