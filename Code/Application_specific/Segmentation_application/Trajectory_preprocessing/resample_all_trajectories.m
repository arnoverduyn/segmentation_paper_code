function [pose_trajectories_kettle,pose_trajectories_bottle] = resample_all_trajectories(pose_trajectories_kettle,pose_trajectories_bottle, parameters)

    for data = 1:parameters.data_size 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        [T,time_new] = resample_temporal_trajectory(pose_trajectory,parameters.dt,parameters.ds);
        pose_trajectories_kettle(data).pose_trajectory = T;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    [T,time_new] = resample_temporal_trajectory(pose_trajectory,parameters.dt,parameters.ds);
    pose_trajectories_bottle.pose_trajectory = T;
   
end