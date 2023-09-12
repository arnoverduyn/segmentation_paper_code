function [pose_trajectory,time_vector,N] = design_pouring_skill_kettle(std_dev_angle,std_dev_pos,dt,ref_point_number)
% This function generates three trajectories for the pouring motion
% performed with a kettle. Each trajectory has a different location of the
% reference point on the kettle.
    
    %% Segment 0: Standstill
    t_segm = 2; % 1 second
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    
    T_segm = zeros(4,4,N_segm);
    for k = 1:N_segm
        T_segm(:,:,k) = eye(4);
    end
    time_vector = time_vector_segm;
    pose_trajectory = T_segm;
    
    
    %% Segment 1: sliding the jug across the table towards himself
    t_segm = 3; % 1.5 seconds
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    vel_segm = 0.3/3*[-1/sqrt(2);-1/sqrt(2);0]; % 30 cm/s
    vel_segm = vel_segm*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    
    T_segm = zeros(4,4,N_segm);
    T_segm(:,:,1) = pose_trajectory(:,:,end);
    for k = 2:N_segm
        T_segm(1:3,1:3,k) = T_segm(1:3,1:3,k-1);
        T_segm(1:3,4,k) = T_segm(1:3,4,k-1) + vel_segm(:,k)*dt;
        T_segm(4,4,k) = 1;
    end
    
    time_vector = [time_vector, time_vector(end) + time_vector_segm(2:end)];
    for k = 2:N_segm
        pose_trajectory(:,:,end+1) = T_segm(:,:,k);
    end

    %% Segment 2: picking up jug
    t_segm = 2; % 1 second
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    omega_segm = 95/180*pi; % 95 degrees per second
    omega_segm = omega_segm/time_vector_segm(end)*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    vel_segm = 0.1; % 10 cm per second
    vel_segm = vel_segm/time_vector_segm(end)*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    pose_screw_axis = eye(4);
    pose_screw_axis(1:3,4) = [-0.4;-0.3;0];
    
    T_segm = zeros(4,4,N_segm);
    T_segm(:,:,1) = pose_trajectory(:,:,end);
    for k = 2:N_segm
        delta_T = eye(4);
        delta_T(1:3,1:3) = rotz(omega_segm(k)*dt*180/pi);
        delta_T(3,4) = vel_segm(k)*dt;
        delta_T = pose_screw_axis*delta_T*inverse_T(pose_screw_axis);
        T_segm(:,:,k) = delta_T*T_segm(:,:,k-1);
    end
    
    time_vector = [time_vector, time_vector(end) + time_vector_segm(2:end)];
    for k = 2:N_segm
        pose_trajectory(:,:,end+1) = T_segm(:,:,k);
    end
    
    %% Segment 3: rotation about z-axis
    t_segm = 4; % 2 seconds
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    omega_segm = 60/180*pi; % 60 degrees per second
    omega_segm = omega_segm/time_vector_segm(end)*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    omega_segm = omega_segm.*[-1/sqrt(2);-1/sqrt(2);0];
    pose_screw_axis = eye(4);
    pose_screw_axis(1:3,4) = [-0.55;-0.13;0.15];
    
    T_segm = zeros(4,4,N_segm);
    T_segm(:,:,1) = pose_trajectory(:,:,end);
    for k = 2:N_segm
        delta_T = eye(4);
        delta_T(1:3,1:3) = expm_rot(omega_segm(:,k),dt);
        delta_T = pose_screw_axis*delta_T*inverse_T(pose_screw_axis);
        T_segm(:,:,k) = delta_T*T_segm(:,:,k-1);
    end
    
    time_vector = [time_vector, time_vector(end) + time_vector_segm(2:end)];
    for k = 2:N_segm
        pose_trajectory(:,:,end+1) = T_segm(:,:,k);
    end
    
    %% Segment 4: Reverse rotation about z-axis
    t_segm = 2; % 1 second
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    omega_segm = -60/180*pi; % 60 degrees per second
    omega_segm = omega_segm/time_vector_segm(end)*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    omega_segm = omega_segm.*[-1/sqrt(2);-1/sqrt(2);0];
    pose_screw_axis = eye(4);
    pose_screw_axis(1:3,4) = [-0.55;-0.13;0.15];
    
    T_segm = zeros(4,4,N_segm);
    T_segm(:,:,1) = pose_trajectory(:,:,end);
    for k = 2:N_segm
        delta_T = eye(4);
        delta_T(1:3,1:3) = expm_rot(omega_segm(:,k),dt);
        delta_T = pose_screw_axis*delta_T*inverse_T(pose_screw_axis);
        T_segm(:,:,k) = delta_T*T_segm(:,:,k-1);
    end
    
    time_vector = [time_vector, time_vector(end) + time_vector_segm(2:end)];
    for k = 2:N_segm
        pose_trajectory(:,:,end+1) = T_segm(:,:,k);
    end
    
    %% Segment 5: putting down jug
    t_segm = 2; % 2 seconds
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    omega_segm = -80/180*pi; % 80 degrees per second
    omega_segm = omega_segm/time_vector_segm(end)*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    vel_segm = -0.1; % 10 cm per second
    vel_segm = vel_segm/time_vector_segm(end)*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    pose_screw_axis = eye(4);
    pose_screw_axis(1:3,4) = [-0.4;-0.3;0];
    
    T_segm = zeros(4,4,N_segm);
    T_segm(:,:,1) = pose_trajectory(:,:,end);
    for k = 2:N_segm
        delta_T = eye(4);
        delta_T(1:3,1:3) = rotz(omega_segm(k)*dt*180/pi);
        delta_T(3,4) = vel_segm(k)*dt;
        delta_T = pose_screw_axis*delta_T*inverse_T(pose_screw_axis);
        T_segm(:,:,k) = delta_T*T_segm(:,:,k-1);
    end
    
    time_vector = [time_vector, time_vector(end) + time_vector_segm(2:end)];
    for k = 2:N_segm
        pose_trajectory(:,:,end+1) = T_segm(:,:,k);
    end
    
    %% Segment 1: sliding the jug away
    t_segm = 2; % 1 second
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    vel_segm = 0.3/2*[0.3;1;0]/norm([0.3;1;0]); % 30 cm/s
    vel_segm = vel_segm*(1-cos(2*pi/time_vector_segm(end)*time_vector_segm));
    
    T_segm = zeros(4,4,N_segm);
    T_segm(:,:,1) = pose_trajectory(:,:,end);
    for k = 2:N_segm
        T_segm(1:3,1:3,k) = T_segm(1:3,1:3,k-1);
        T_segm(1:3,4,k) = T_segm(1:3,4,k-1) + vel_segm(:,k)*dt;
        T_segm(4,4,k) = 1;
    end
    
    time_vector = [time_vector, time_vector(end) + time_vector_segm(2:end)];
    for k = 2:N_segm
        pose_trajectory(:,:,end+1) = T_segm(:,:,k);
    end
    
    %% Segment end: Standstill
    t_segm = 0.5; % 1 second
    time_vector_segm = 0:dt:t_segm;
    N_segm = length(time_vector_segm);
    
    T_segm(:,:,1) = pose_trajectory(:,:,end);
    for k = 2:N_segm
        T_segm(:,:,k) = T_segm(:,:,k-1);
    end
    
    time_vector = [time_vector, time_vector(end) + time_vector_segm(2:end)];
    for k = 2:N_segm
        pose_trajectory(:,:,end+1) = T_segm(:,:,k);
    end
    
    N = length(time_vector);

    %% Change ref point
    % 0 -> ref point near Center of Mass
    % 1 -> ref point near handle
    % 2 -> ref point near spout
    if ref_point_number > 0.5
        Delta_T = eye(4);
        if ref_point_number == 1
            Delta_T(1:3,4) = rotz(-35)*[0;-0.088;0.036];
        elseif ref_point_number == 2
            Delta_T(1:3,4) = rotz(-35)*[0;0.055;0.0372];
        end
        for k = 1:N
            pose_trajectory(:,:,k) = pose_trajectory(:,:,k)*Delta_T;
        end
    end

    %% Add noise
    for k = 1:N
        theta_dist = std_dev_angle*randn(3,1);
        pos_dist = std_dev_pos*randn(3,1);
        pose_trajectory(1:3,1:3,k) = expm(skew(theta_dist))*pose_trajectory(1:3,1:3,k);
        pose_trajectory(1:3,4,k) = pose_trajectory(1:3,4,k) + pos_dist;
    end

end