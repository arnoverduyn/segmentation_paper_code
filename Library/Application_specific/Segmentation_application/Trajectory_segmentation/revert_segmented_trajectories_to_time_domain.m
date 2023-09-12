function [pose_trajectories_kettle,pose_trajectories_bottle] = revert_segmented_trajectories_to_time_domain(pose_trajectories_kettle,pose_trajectories_bottle,parameters)
    for data = 1:parameters.data_size
        associated_cluster_indices = [NaN,NaN,pose_trajectories_kettle(data).associated_cluster_indices,NaN];
        s_t = pose_trajectories_kettle(data).s_t;
        N_t = size(s_t,2);
        xi = pose_trajectories_kettle(data).xi;
        N_s = size(xi,2);
        associated_cluster_indices_wrt_time = NaN(1,N_t);
        counter = 1;
        for k = 1:N_s
            while s_t(counter) < xi(k)
                associated_cluster_indices_wrt_time(counter) = associated_cluster_indices(k);
                counter = counter + 1;
            end
        end    
        pose_trajectories_kettle(data).associated_cluster_indices = associated_cluster_indices_wrt_time;
    end
end