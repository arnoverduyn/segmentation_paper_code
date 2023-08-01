function generic_clusters = learning_of_shapes(pose_trajectories_kettle,parameters,bools)
% This function learns geometric trajectory-shape primitives from the data
% using the incremental clustering approach explained in the paper. This
% function consists of three steps:
% 1) Initialization: Calculation of total mean and standard deviation of
%    all the trajectory shapes within the data. This mean and standard
%    deviation is used as an initial guess for the mean and standard deviation
%    of the first cluster.
% 2) Ordering of the sample points based on the calculated mean and
%    standard deviation. (Only usable for off-line applications)
% 3) Incremental clustering
% 4) Outlier removal: remove sparse clusters that do not represent a
%    significant percentage of the data
    
    % Calculate total mean and std_dev (all datapoints)
    if bools.print_level; disp('   Calculate statistics of total batch (3 trajectories of kettle)...'); end

    L = parameters.L;
    N1 = size(pose_trajectories_kettle(1).shape_matrix,3);
    N2 = size(pose_trajectories_kettle(2).shape_matrix,3);
    N3 = size(pose_trajectories_kettle(3).shape_matrix,3);
    N = N1 + N2 + N3;
    batch = zeros(6,3,N);
    batch(:,:,1:N1) = pose_trajectories_kettle(1).shape_matrix;
    batch(:,:,N1+1:N1+N2) = pose_trajectories_kettle(2).shape_matrix; 
    batch(:,:,N1+N2+1:N1+N2+N3) = pose_trajectories_kettle(3).shape_matrix; 

    % Find global viewpoint (all datapoints) (not that important)
    vel_vectors = zeros(3,2*N);
    for n = 1:N
        vel_vectors(:,1 + 2*(n-1)) = L*batch(1:3,2,n);
        vel_vectors(:,2 + 2*(n-1)) = batch(4:6,2,n);
    end
    [U,~,~] = svd(vel_vectors*vel_vectors');
    if det(U) < 0
        U(1:3,3) = -U(1:3,3);
    end

    % Calculate mean shape descriptor (all datapoints)
    sum_twists = zeros(6,3);
    for n = 1:N
        current_omega = U'*batch(1:3,1:3,n);
        current_vel = U'*batch(4:6,1:3,n);
        sum_twists = sum_twists + [L*current_omega;current_vel];
    end
    average_shape = sum_twists/N;

    % Calculate standard deviation shape descriptor (all datapoints)
    variance = 0;
    for n = 1:N
        A_ = [L*batch(1:3,1:3,n),batch(4:6,1:3,n)];
        B_ = [L*average_shape(1:3,:),average_shape(4:6,:)];
        C = A_*B_';
        [U,~,V] = svd(C);
        R = V*U';
        if det(R) < 0
            U(1:3,3) = -U(1:3,3);
            R = V*U';
        end
        diff_matrix = (R*A_-B_).^2;
        variance = variance + sum(diff_matrix,'all');
    end
    std_dev = sqrt(variance/(N-1));    

    batch_unordered = batch;

    %% Order datapoints before clustering
    if bools.order_learning_data
        if bools.print_level; disp('   Order batch sample points before incremental clustering...'); end
        
        batch_ordered = zeros(6,3,N);
        included_shapes = average_shape;
        counter = 1;
        percentiles = round((N)/5);
        for j = 1:N-1
    
            if bools.print_level
                if j == percentiles*counter
                    disp(strcat(['      Progress: ' num2str(round(j/(N-1)*10)*10) '%']))
                    counter = counter+1;
                end
            end
    
            shortest_distance = inf;
            nb_comparisons = size(included_shapes,3);
            for n = 1:N
                if ~isnan(batch(1,1,n))
                    summed_distance = 0;
                    A_ = [L*batch(1:3,:,n),batch(4:6,:,n)];
                    for k = 1:nb_comparisons
                        B_ = [L*included_shapes(1:3,:,k),included_shapes(4:6,:,k)];
                        C = A_*B_';
                        [U,~,V] = svd(C);
                        R = V*U';
                        if det(R) < 0
                            U(1:3,3) = -U(1:3,3);
                            R = V*U';
                        end
                        diff_matrix = (R*A_-B_).^2;
                        summed_distance = summed_distance + sum(diff_matrix,'all');
                    end
                    if summed_distance < shortest_distance
                        shortest_distance = summed_distance;
                        closest_sample = n;
                    end
                end
            end
            batch_ordered(:,:,j) = batch(:,:,closest_sample);
            included_shapes(:,:,j+1) = batch(:,:,closest_sample);
            % pop this sample
            batch(:,:,closest_sample) = nan(6,3);
        end
    
        % add last sample
        for n = 1:N
            if ~isnan(batch(1,1,n))
                 last_sample = batch(:,:,n);
            end
        end
        batch_ordered(:,:,N) = last_sample;
        init_std_dev = std_dev; 
    else
        batch_ordered = batch_unordered;
        init_std_dev = std_dev/10; 
    end

    %% Incremental clusering algorithm
    if bools.print_level; disp('   Incremental clustering of trajectory shapes...'); end

    shape_matrix = batch_ordered;
    min_Q = parameters.min_Q; 
    stopping_criterion = 10^(-6); % when relative change in std_dev is small
    max_iterations = 500;
    relative_change = inf;
    iteration = 1;
    clusters = struct();
    while relative_change > stopping_criterion && iteration < max_iterations
        clusters(1).mean = average_shape;
        clusters(1).nb_datapoints = 1;
        clusters(1).std_dev = init_std_dev;
        nb_clusters = 1;
        for n = 1:N
            current_shape = shape_matrix(:,:,n);
            shortest_distance = inf;
            for k = 1:nb_clusters
                current_cluster_mean = clusters(k).mean;
                A_ = [L*current_shape(1:3,:),current_shape(4:6,:)];
                B_ = [L*current_cluster_mean(1:3,:),current_cluster_mean(4:6,:)];
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
                    closest_cluster_nb = k;
                    current_shape_rotated(1:3,1:3) = R*current_shape(1:3,1:3);
                    current_shape_rotated(4:6,1:3) = R*current_shape(4:6,1:3);
                end
            end
            closest_cluster_std_dev = clusters(closest_cluster_nb).std_dev;
            if shortest_distance < 3*closest_cluster_std_dev 
                % number of clusters still valid
                nb_datapoints = clusters(closest_cluster_nb).nb_datapoints;
                old_mean = clusters(closest_cluster_nb).mean;
                old_std_dev = clusters(closest_cluster_nb).std_dev;
                % Update mean
                clusters(closest_cluster_nb).mean = (nb_datapoints*old_mean + current_shape_rotated)/(nb_datapoints+1);
                % Update variance
                clusters(closest_cluster_nb).std_dev = sqrt(nb_datapoints/(nb_datapoints+1)*(old_std_dev^2 + (shortest_distance)^2/(nb_datapoints+1)));
                % Update nb_datapoints
                clusters(closest_cluster_nb).nb_datapoints = nb_datapoints+1;
            else
                % introduce new cluster
                nb_clusters = nb_clusters+1;
                clusters(nb_clusters).mean = current_shape;
                clusters(nb_clusters).std_dev = init_std_dev;
                clusters(nb_clusters).nb_datapoints = 1;
            end   
        end

        % update initial uncertainty
        init_std_dev_old = init_std_dev;
        mean_std_dev_of_all_clusters = 0;
        for k = 1:nb_clusters
            mean_std_dev_of_all_clusters = mean_std_dev_of_all_clusters + clusters(k).nb_datapoints*clusters(k).std_dev;
        end
        mean_std_dev_of_all_clusters = mean_std_dev_of_all_clusters/N/nb_clusters;
        init_std_dev = mean_std_dev_of_all_clusters + min_Q;
        relative_change = abs(init_std_dev_old-init_std_dev)/init_std_dev_old;

        iteration = iteration + 1;
    end

    %% Dimensionality reduction step: reduce the number of clusters
    if bools.print_level; disp('   Remove clusters representing outliers...'); end

    generic_clusters = struct();
    cluster_nb = 1;
    for k = 1:nb_clusters
        if clusters(k).nb_datapoints/N > parameters.outlier_percentage/100 % cluster represents at least x% of data
            generic_clusters(cluster_nb).nb_datapoints = clusters(k).nb_datapoints;
            generic_clusters(cluster_nb).mean = clusters(k).mean;
            generic_clusters(cluster_nb).std_dev = clusters(k).std_dev;
            cluster_nb = cluster_nb + 1;
        end
    end
    nb_clusters = cluster_nb-1;
    if bools.print_level; disp(strcat(['   Number of generic clusters: ' num2str(nb_clusters)])); end


end