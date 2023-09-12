function [pose_trajectories_kettle,pose_trajectories_bottle] = segment_all_trajectories(generic_clusters,pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools)

    for data = 1:parameters.data_size
        input_trajectory = pose_trajectories_kettle(data);
        segmented_trajectory = segment_trajectory(generic_clusters,input_trajectory,bools,parameters);
        pose_trajectories_kettle(data).associated_cluster_indices = segmented_trajectory.associated_cluster_indices;
        pose_trajectories_kettle(data).segments = segmented_trajectory.segments;
    end
    input_trajectory = pose_trajectories_bottle;
    segmented_trajectory = segment_trajectory(generic_clusters,input_trajectory,bools,parameters);
    pose_trajectories_bottle.associated_cluster_indices = segmented_trajectory.associated_cluster_indices;
    pose_trajectories_bottle.segments = segmented_trajectory.segments;

end