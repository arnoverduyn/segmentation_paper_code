function [pose_trajectories_kettle,pose_trajectories_bottle] = calculate_geom_progress_rate_all_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools)

 %% Calculate instantaneous ref point and progress rate
 
% For trajectories with kettle
for data = 1:parameters.data_size
    pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
    twist = pose_trajectories_kettle(data).twist;
    xi_dot = calculate_geom_progress_rate(pose_trajectory,twist,parameters,bools);
    pose_trajectories_kettle(data).xi_dot = xi_dot;
end

% For trajectory with bottle
pose_trajectory = pose_trajectories_bottle.pose_trajectory;
twist = pose_trajectories_bottle.twist;
xi_dot = calculate_geom_progress_rate(pose_trajectory,twist,parameters,bools);
pose_trajectories_bottle.xi_dot = xi_dot;

end