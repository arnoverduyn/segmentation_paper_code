function [input_trajectory] = segment_trajectory(clusters,input_trajectory,bools,parameters)
% This function uses the learned trajectory-shape primitives to segment the
% input pose trajectory conform the approach explained in the paper.

    if bools.print_level; disp('   Segmenting trajectory...'); end
    
    L = parameters.L;
    shape_matrix = input_trajectory.shape_matrix;
    nb_clusters = length(clusters);
    closest_cluster = NaN;
    N = size(shape_matrix,3);
    associated_cluster_indices = zeros(1,N);
    for n = 1:N
        current_shape = shape_matrix(:,:,n);
        shortest_distance = inf;
        for k = 1:nb_clusters 
            current_cluster_mean = clusters(k).mean;
            A_ = [L*current_cluster_mean(1:3,:),current_cluster_mean(4:6,:)];
            B_ = [L*current_shape(1:3,:),current_shape(4:6,:)];
            C = A_*B_';
            [U,~,V] = svd(C);
            R = V*U';
            if det(R) < 0
                U(1:3,3) = -U(1:3,3);
                R = V*U';
            end
            diff_matrix = (R*A_-B_).^2;
            distance_to_mean = sqrt(sum(diff_matrix,'all'));
            if distance_to_mean < shortest_distance
                shortest_distance = distance_to_mean;
                closest_cluster = k;
            end
        end
        closest_cluster_std_dev = clusters(closest_cluster).std_dev;
        if shortest_distance > 3*closest_cluster_std_dev
            associated_cluster_indices(n) = NaN;
        else
            associated_cluster_indices(n) = closest_cluster;
        end

    end

    %% Grouping of subsequent sample points associated to the same cluster
    
    % look for initial index different from NaN
    k = 1;
    while isnan(associated_cluster_indices(k))
        k = k+1;
    end
    
    % start new segment
    nb_segments = 1;
    segments(nb_segments).datapoints = k;
    segments(nb_segments).cluster_nb = associated_cluster_indices(k);
    k = k+1;
    while k < N
        if associated_cluster_indices(k) == associated_cluster_indices(k-1) % increase the number of samples in the current segment
            segments(nb_segments).datapoints = k;
        elseif ~isnan(associated_cluster_indices(k)) % Directly start new segment
            nb_segments = nb_segments + 1;
            segments(nb_segments).datapoints = k;
            segments(nb_segments).cluster_nb = associated_cluster_indices(k);
        else  % Search for new segment
            while k <= N && isnan(associated_cluster_indices(k)) 
                k = k+1;
            end
            if k <= N
                % start new segment
                nb_segments = nb_segments + 1;
                segments(nb_segments).datapoints = k;
                segments(nb_segments).cluster_nb = associated_cluster_indices(k);
            end
        end
        k = k+1;
    end
    
%     %% remove small segments
%     long_segments = struct();
%     nb_long_segments = 1;
%     for k = 1:nb_segments
%         segment_N = segments(k).datapoints;
%         if segment_N > 5/100*N % sufficiently long segment
%             long_segments(nb_long_segments).datapoints = segments(k).datapoints;
%             long_segments(nb_long_segments).cluster_nb = segments(k).cluster_nb;
%             nb_long_segments = nb_long_segments + 1;
%         end
%     end
%     nb_long_segments = nb_long_segments-1;
%     
    input_trajectory.associated_cluster_indices = associated_cluster_indices;
    input_trajectory.segments = segments;
    
    if bools.print_level; disp(strcat(['   Number of detected segments: ' num2str(nb_segments)])); end
    
end