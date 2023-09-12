function [pose_trajectories_kettle,pose_trajectories_bottle] = construct_shape_matrix_all(pose_trajectories_kettle,pose_trajectories_bottle,parameters)
    
    % for trajectories kettle
    for data = 1:parameters.data_size 
        geom_twist = pose_trajectories_kettle(data).geom_twist;
        N = size(geom_twist,2);
        shape_matrix = zeros(6,3,N);
        for j = 2:N-1
            shape_matrix(:,1,j) = geom_twist(:,j-1);
            shape_matrix(:,2,j) = geom_twist(:,j);
            shape_matrix(:,3,j) = geom_twist(:,j+1);
        end
        shape_matrix = shape_matrix(:,:,2:N-1); % remove first and last sample
        pose_trajectories_kettle(data).shape_matrix = shape_matrix;
    end
    
    % for trajectories bottle
    geom_twist = pose_trajectories_bottle.geom_twist;
    N = size(geom_twist,2);
    shape_matrix = zeros(6,3,N);
    for j = 2:N-1
        shape_matrix(:,1,j) = geom_twist(:,j-1);
        shape_matrix(:,2,j) = geom_twist(:,j);
        shape_matrix(:,3,j) = geom_twist(:,j+1);
    end
    shape_matrix = shape_matrix(:,:,2:N-1); % remove first and last sample
    pose_trajectories_bottle.shape_matrix = shape_matrix;
    
end