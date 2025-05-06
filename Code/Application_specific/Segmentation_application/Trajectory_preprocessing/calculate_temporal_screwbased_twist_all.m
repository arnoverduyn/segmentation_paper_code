function [pose_trajectories_kettle,pose_trajectories_bottle] = calculate_temporal_screwbased_twist_all(pose_trajectories_kettle,pose_trajectories_bottle,parameters)

    for data = 1:parameters.data_size 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        [~,~,twist_ref] = calculate_geom_progress_rate_screwbased_from_pose(pose_trajectory,parameters.ds,parameters.L);
        pose_trajectories_kettle(data).geom_twist = twist_ref;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    [~,~,twist_ref] = calculate_geom_progress_rate_screwbased_from_pose(pose_trajectory,parameters.ds,parameters.L);
    pose_trajectories_bottle.geom_twist = twist_ref;

end