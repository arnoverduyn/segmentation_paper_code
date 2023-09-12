function [pose_trajectories_kettle,pose_trajectories_bottle] = calculate_twist_from_geometric_pose_trajectory_all(pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools)

    for data = 1:parameters.data_size 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        twist = calculate_twist_from_geometric_pose_trajectory(pose_trajectory,bools,parameters);
        pose_trajectories_kettle(data).geom_twist = twist;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    twist = calculate_twist_from_geometric_pose_trajectory(pose_trajectory,bools,parameters);
    pose_trajectories_bottle.geom_twist = twist;

end