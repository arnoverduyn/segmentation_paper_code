function [pose_trajectories_kettle,pose_trajectories_bottle] = reparametrize_all_trajectories_to_geom_domain(pose_trajectories_kettle,pose_trajectories_bottle,parameters)

    for data = 1:parameters.data_size 
        pose_trajectory = pose_trajectories_kettle(data).pose_trajectory;
        xi_dot = pose_trajectories_kettle(data).xi_dot;
        [T_s,xi,s_t] = reparametrize_to_geom_domain(pose_trajectory,parameters.dt,xi_dot,parameters.ds);
        pose_trajectories_kettle(data).pose_trajectory = T_s;
        pose_trajectories_kettle(data).xi = xi;
        pose_trajectories_kettle(data).s_t = s_t;
    end
    pose_trajectory = pose_trajectories_bottle.pose_trajectory;
    xi_dot = pose_trajectories_bottle.xi_dot;
    [T_s,xi,s_t] = reparametrize_to_geom_domain(pose_trajectory,parameters.dt,xi_dot,parameters.ds);
    pose_trajectories_bottle.pose_trajectory = T_s;
    pose_trajectories_bottle.xi = xi;
    pose_trajectories_bottle.s_t = s_t;
        
end