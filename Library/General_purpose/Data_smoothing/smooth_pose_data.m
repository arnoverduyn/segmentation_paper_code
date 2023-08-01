function [T_smooth,twist,twistdot] = smooth_pose_data(T,params) 
% This function smooths the input pose trajectory using a Kalman smoother
% with white noise jerk model
% INPUT: T (4x4xN)
% OUTPUT: T_smooth (4x4xN) -> smoothed pose trajectory
%         twist    (6xN)   -> smooth pose twist
%         twistdot (6xN)   -> smooth derivative of the pose twist

    % Kalman smoother
    ROT = T(1:3,1:3,:);
    pos = squeeze(T(1:3,4,:));
    q = rot2quat(ROT);
    raw_data = [pos',q];

    [smooth_data,twist,twistdot] = preprocess_pose_data(raw_data,params);

    % transform back to pose data
    N = length(pos(1,:));
    ROT = zeros(3,3,N);
    T_smooth = zeros(4,4,N);
    for j = 1:N
        ROT(:,:,j) = quat2rotm([smooth_data(j,7),smooth_data(j,4:6)]);
        T_smooth(1:3,1:3,j) = ROT(:,:,j);
        T_smooth(1:3,4,j) = smooth_data(j,1:3)';
        T_smooth(4,4,j) = 1;
    end
end
