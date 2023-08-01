function [pose_trajectories_kettle,pose_trajectories_bottle] = generate_simulation_data_trajectories(parameters)
% This function generates simulation data of the pouring motions performed
% with a kettle and a bottle

    % trajectories with kettle
    pose_trajectories_kettle = struct();
    for ref_point_number = 0:2 % 
        [pose_trajectory,~,]  = design_pouring_skill_kettle(parameters.std_dev_angle,parameters.std_dev_pos,parameters.dt,ref_point_number);
        pose_trajectories_kettle(ref_point_number+1).pose_trajectory = pose_trajectory;
    end

    % trajectory with bottle
    [pose_trajectory,~,~]  = design_pouring_skill_bottle(parameters.std_dev_angle,parameters.std_dev_pos,parameters.dt);
    pose_trajectories_bottle.pose_trajectory = pose_trajectory;
    
end