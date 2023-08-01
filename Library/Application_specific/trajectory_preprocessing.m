function [pose_trajectories_kettle,pose_trajectories_bottle] = trajectory_preprocessing(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,bools,parameters)
% This function preprocesses the simulated trajectories as explained in the
% paper. The different steps include:
% 1) Smoothing of the trajectories using a Kalman smoother with white noise
% jerk model
% 2) Calculation of the proposed geometric progress rate for rigid body
% trajectories
% 3) Reparameterization of the simulated temporal rigid body trajectories
% to a geometric domain using linear interpolation
% 4) Calculation of the geometric twists along the trajectory
% 5) Construction of the local geometric trajectory-shape descriptor S
% along the trajectory

    % smooth all trajectories
    if bools.print_level; disp('   Smooth rigid body trajectory data...');end
    for data = 1:3 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        [smooth_pose_trajectory,Twist,~] = ...
            smooth_pose_data(pose_trajectory,parameters);
        pose_trajectories_kettle(data).pose_trajectory = smooth_pose_trajectory;
        pose_trajectories_kettle(data).twist = Twist;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    [smooth_pose_trajectory,Twist,~] = ...
        smooth_pose_data(pose_trajectory,parameters);
    pose_trajectories_bottle.pose_trajectory = smooth_pose_trajectory;
    pose_trajectories_bottle.twist = Twist;

    if bools.bool_plot_all_figures
        plot_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup)
    end
    if bools.bool_show_intermediate_movies; movie_input_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,5,0.1); end

    % calculate geometric trajectory-shape descriptors of all trajectories
    if bools.print_level; disp('   Model geometric trajectory-shape descriptors for all trajectories...'); end

    %% Calculate instantaneous ref point and progress rate
    for data = 1:3 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        twist = pose_trajectories_kettle.twist;
        [xi_dot,p_ref,twist_ref] = calculate_geom_progress_rate_from_twist(pose_trajectory,twist',parameters.L);
        pose_trajectories_kettle(data).xi_dot = xi_dot;
        pose_trajectories_kettle(data).p_ref = p_ref;
        pose_trajectories_kettle(data).twist_ref = twist_ref;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    twist = pose_trajectories_bottle.twist;
    [xi_dot,p_ref,twist_ref] = calculate_geom_progress_rate_from_twist(pose_trajectory,twist',parameters.L);
    pose_trajectories_bottle.xi_dot = xi_dot;
    pose_trajectories_bottle.p_ref = p_ref;
    pose_trajectories_bottle.twist_ref = twist_ref;

    if bools.bool_show_intermediate_movies; movie_input_trajectories_with_ref_point(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup); end

    %% Reparametrize pose trajectory to the geometric domain
    for data = 1:3 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        xi_dot = pose_trajectories_kettle(data).xi_dot;
        [T_s,xi] = reparametrize_to_geom_domain(pose_trajectory,parameters.dt,xi_dot,parameters.ds);
        pose_trajectories_kettle(data).pose_trajectory = T_s;
        pose_trajectories_kettle(data).xi = xi;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    xi_dot = pose_trajectories_bottle.xi_dot;
    [T_s,xi] = reparametrize_to_geom_domain(pose_trajectory,parameters.dt,xi_dot,parameters.ds);
    pose_trajectories_bottle.pose_trajectory = T_s;
    pose_trajectories_bottle.xi = xi;

    if bools.bool_plot_all_figures
        plot_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup)
    end
    if bools.bool_show_intermediate_movies; movie_input_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,2,0.4); end


    %% Calculate the geometric twists along the trajectory
    for data = 1:3 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        [~,~,twist_ref] = calculate_geom_progress_rate_from_pose(pose_trajectory,parameters.ds,parameters.L);
        N = size(twist_ref,2) + 1;
        for k = 1:N-1 % normalize geometric twists
            weighted_norm = sqrt(parameters.L^2*twist_ref(1:3,k)'*twist_ref(1:3,k) + twist_ref(4:6,k)'*twist_ref(4:6,k));
            twist_ref(:,k) = twist_ref(:,k)/weighted_norm;
        end
        pose_trajectories_kettle(data).geom_twist = twist_ref;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    [~,~,twist_ref] = calculate_geom_progress_rate_from_pose(pose_trajectory,parameters.ds,parameters.L);
    N = size(twist_ref,2) + 1;
    for k = 1:N-1 % normalize geometric twists
        weighted_norm = sqrt(parameters.L^2*twist_ref(1:3,k)'*twist_ref(1:3,k) + twist_ref(4:6,k)'*twist_ref(4:6,k));
        twist_ref(:,k) = twist_ref(:,k)/weighted_norm;
    end
    pose_trajectories_bottle.geom_twist = twist_ref;

    %% Construct the geometric trajectory-shape descriptor matrix
    for data = 1:3 
        geom_twist = pose_trajectories_kettle(data).geom_twist;
        N = size(geom_twist,2);
        shape_matrix = zeros(6,3,N);
        for j = 2:N-1
            shape_matrix(:,1,j) = geom_twist(:,j-1);
            shape_matrix(:,2,j) = geom_twist(:,j);
            shape_matrix(:,3,j) = geom_twist(:,j+1);
        end
        shape_matrix = shape_matrix(:,:,2:N-1); % remove first and last sample
        pose_trajectories_kettle(data).shape_matrix = shape_matrix;
    end
    geom_twist = pose_trajectories_bottle.geom_twist;
    N = size(geom_twist,2);
    shape_matrix = zeros(6,3,N);
    for j = 2:N-1
        shape_matrix(:,1,j) = geom_twist(:,j-1);
        shape_matrix(:,2,j) = geom_twist(:,j);
        shape_matrix(:,3,j) = geom_twist(:,j+1);
    end
    shape_matrix = shape_matrix(:,:,2:N-1); % remove first and last sample
    pose_trajectories_bottle.shape_matrix = shape_matrix;

end