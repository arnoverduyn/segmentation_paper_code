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
    
    %% smooth all trajectories
    if bools.print_level; disp('   Smooth rigid body trajectory data...');end
    [pose_trajectories_kettle,pose_trajectories_bottle] = smooth_all_data(pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools);

    if ~strcmp(bools.data_type,'real') 
        if bools.bool_plot_all_figures; plot_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,'smoothed trajectories'); end
    end

    %% calculate geometric trajectory-shape descriptors of all trajectories
    if bools.print_level; disp('   Model geometric trajectory-shape descriptors for all trajectories...'); end

    if strcmp(bools.progress_domain,'geometric')
        % Calculate geometric progress rate
        [pose_trajectories_kettle,pose_trajectories_bottle] = calculate_geom_progress_rate_all_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools);

        % Reparametrize pose trajectory to the geometric domain
        [pose_trajectories_kettle,pose_trajectories_bottle] = reparametrize_all_trajectories_to_geom_domain(pose_trajectories_kettle,pose_trajectories_bottle,parameters);

        if ~strcmp(bools.data_type,'real') 
            if bools.bool_plot_all_figures; plot_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,'reparameterized trajectories'); end
        end
        
        % Calculate the geometric twists along the trajectory
        [pose_trajectories_kettle,pose_trajectories_bottle] = calculate_twist_from_geometric_pose_trajectory_all(pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools);
        
    else
        % resample data to higher timestep (larger moving window over which the trajectory shape is defined)
        [pose_trajectories_kettle,pose_trajectories_bottle] = resample_all_trajectories(pose_trajectories_kettle,pose_trajectories_bottle, parameters);
        
        % Calculate the temporal twists along the trajectory
        [pose_trajectories_kettle,pose_trajectories_bottle] = calculate_temporal_screwbased_twist_all(pose_trajectories_kettle,pose_trajectories_bottle,parameters);
    end
    
    % Construct the trajectory-shape descriptor matrix
    [pose_trajectories_kettle,pose_trajectories_bottle] = construct_shape_matrix_all(pose_trajectories_kettle,pose_trajectories_bottle,parameters);

end