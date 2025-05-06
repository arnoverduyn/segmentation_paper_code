function [pose_trajectories_kettle,pose_trajectories_bottle] = smooth_all_data(pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools)

    for data = 1:parameters.data_size
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
    
end