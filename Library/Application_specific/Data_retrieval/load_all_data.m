function [pose_trajectories_kettle,pose_trajectories_bottle] = load_all_data(parameters,bools,kettle,bottle,table,cup)
    if strcmp(bools.data_type, 'real')
        for k = 1:6 % six trials
            [pose_trajectory,~] = load_recorded_pouring_motion(k);
            pose_trajectories_kettle(k).pose_trajectory = pose_trajectory;
        end
        pose_trajectories_bottle.pose_trajectory = pose_trajectory; % will not be used
    elseif strcmp(bools.data_type, 'new_simulation')
        [pose_trajectories_kettle,pose_trajectories_bottle] = generate_simulation_data_trajectories(parameters);
    else
        simulation_data = load('Data/simulation_data/simulation_data.mat');
        pose_trajectories_kettle = simulation_data.simulation_data.pose_trajectories_kettle;
        pose_trajectories_bottle = simulation_data.simulation_data.pose_trajectories_bottle;
    end
    
    % save simulation data (to be able to retrieve it later)
    save_simulation_data(pose_trajectories_kettle,pose_trajectories_bottle,bools)
    
    % plotting
    if bools.bool_plot_all_figures 
        if strcmp(bools.data_type,'real')
            plot_real_data(pose_trajectories_kettle,kettle,table,cup)
        else
            plot_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup)
            if bools.bool_show_intermediate_movies; movie_input_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,10,0.1); end
        end
    end

end