function [pose_trajectory,time_vector,N] = design_translational_trajectory(std_dev_angle,std_dev_pos,dt)
% This function generates a translational motion that is used for the
% calibration procedure for L. 
    
    %% Linear translation parallel to x-axis
    t_segm = 2; % 2 seconds
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    vel_segm = [0.3;0;0]; % 20 cm/s
    
    T_segm = zeros(4,4,N_segm);
    T_segm(:,:,1) = eye(4);
    for k = 2:N_segm
        T_segm(1:3,1:3,k) = T_segm(1:3,1:3,k-1);
        T_segm(1:3,4,k) = T_segm(1:3,4,k-1) + vel_segm*dt;
        T_segm(4,4,k) = 1;
    end
    
    time_vector = time_vector_segm;
    pose_trajectory = T_segm;
    
    %% Add noise
    N = length(time_vector);
    for k = 1:N
        theta_dist = std_dev_angle*randn(3,1);
        pos_dist = std_dev_pos*randn(3,1);
        pose_trajectory(1:3,1:3,k) = expm(skew(theta_dist))*pose_trajectory(1:3,1:3,k);
        pose_trajectory(1:3,4,k) = pose_trajectory(1:3,4,k) + pos_dist;
    end
    
end