function xi_dot = calculate_geom_progress_rate(pose_trajectory,twist,parameters,bools)
    if strcmp(bools.progress_type,'screw_based_new')
        [xi_dot,~,~] = calculate_geom_progress_rate_screwbased_from_twist(pose_trajectory,twist',parameters.L);
    elseif strcmp(bools.progress_type,'arclength')
        xi_dot = calculate_geom_progress_rate_based_on_arclength_from_twist(pose_trajectory,twist');
    elseif strcmp(bools.progress_type,'angle')
        xi_dot = calculate_geom_progress_rate_based_on_angle_from_twist(pose_trajectory,twist');
    elseif strcmp(bools.progress_type,'combined')
        xi_dot = calculate_geom_progress_rate_combined_from_twist(pose_trajectory,twist',parameters.L);
    elseif strcmp(bools.progress_type,'screw_based_old')
        xi_dot = calculate_geom_progress_rate_Joris_from_twist(pose_trajectory,twist',parameters.L);
    end
end