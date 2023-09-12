function twist = calculate_twist_from_geometric_pose_trajectory(pose_trajectory,bools,parameters)

    if strcmp(bools.progress_type,'screw_based_new')
        [~,~,twist] = calculate_geom_progress_rate_screwbased_from_pose(pose_trajectory,parameters.ds,parameters.L);
        twist = normalize_screwbased_twist(twist,parameters);
    elseif strcmp(bools.progress_type,'screw_based_old')
        [~,~,twist] = calculate_geom_progress_rate_Joris_from_pose(pose_trajectory,parameters.ds,parameters.L);
        twist = normalize_screwbased_Joris_twist(twist,parameters);
    elseif strcmp(bools.progress_type,'combined')
        twist = calculate_twist_finite_differences(pose_trajectory,parameters);
        twist = normalize_twist_combined(twist, parameters);
    elseif strcmp(bools.progress_type,'arclength')
        twist = calculate_twist_finite_differences(pose_trajectory,parameters);
        twist = normalize_twist_arclength(twist);
    elseif strcmp(bools.progress_type,'angle')
        twist = calculate_twist_finite_differences(pose_trajectory,parameters);
        twist = normalize_twist_angle(twist);
    end   
end