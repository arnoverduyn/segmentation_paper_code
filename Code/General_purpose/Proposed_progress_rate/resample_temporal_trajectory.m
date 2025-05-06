function [T,time_new] = resample_temporal_trajectory(T,dt,dt_new)
% This function reparametrizes the input temporal pose trajectory to a
% geometric domain, using the input geometric progress rate. The
% reparameterization is performed by linear interpolation.
% INPUT: T (4x4xN)    -> Input temporal pose trajectory
%      : dt           -> old time step [s]
%      : dt_new       -> new time step [s]
% OUTPUT: T (4x4xN)   -> Resampled temporal pose trajectory
%       : time_new (1xN) -> Corresponding time vector 

N = size(T,3);
time_old = 0:dt:(N-1)*dt;
time_new = 0:dt_new:time_old(end);
T = interpT(time_old,T,time_new);

end