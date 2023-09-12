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
    if strcmp(bools.data_type,'real')
        N1 = size(pose_trajectories_kettle(1).shape_matrix,3);
        N2 = size(pose_trajectories_kettle(2).shape_matrix,3);
        N3 = size(pose_trajectories_kettle(3).shape_matrix,3);
        N4 = size(pose_trajectories_kettle(4).shape_matrix,3);
        N5 = size(pose_trajectories_kettle(5).shape_matrix,3);
        N6 = size(pose_trajectories_kettle(6).shape_matrix,3);
        N = N1 + N2 + N3 + N4 + N5 + N6;
        batch = zeros(6,3,N);
        batch(:,:,1:N1) = pose_trajectories_kettle(1).shape_matrix;
        batch(:,:,N1+1:N1+N2) = pose_trajectories_kettle(2).shape_matrix; 
        batch(:,:,N1+N2+1:N1+N2+N3) = pose_trajectories_kettle(3).shape_matrix; 
        batch(:,:,N1+N2+N3+1:N1+N2+N3+N4) = pose_trajectories_kettle(4).shape_matrix; 
        batch(:,:,N1+N2+N3+N4+1:N1+N2+N3+N4+N5) = pose_trajectories_kettle(5).shape_matrix; 
        batch(:,:,N1+N2+N3+N4+N5+1:N1+N2+N3+N4+N5+N6) = pose_trajectories_kettle(6).shape_matrix; 
    else
        N1 = size(pose_trajectories_kettle(1).shape_matrix,3);
        N2 = size(pose_trajectories_kettle(2).shape_matrix,3);
        N3 = size(pose_trajectories_kettle(3).shape_matrix,3);
        N = N1 + N2 + N3;
        batch = zeros(6,3,N);
        batch(:,:,1:N1) = pose_trajectories_kettle(1).shape_matrix;
        batch(:,:,N1+1:N1+N2) = pose_trajectories_kettle(2).shape_matrix; 
        batch(:,:,N1+N2+1:N1+N2+N3) = pose_trajectories_kettle(3).shape_matrix; 
    end
    
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

    init_std_dev = parameters.min_Q; 

    %% Incremental clusering algorithm
    if bools.print_level; disp('   Incremental clustering of trajectory shapes...'); end
    
    shape_matrix = batch;
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
                if strcmp(bools.progress_domain,'geometric')
                    % Re-normalize mean %% DO FOR ALL METHODS!!!
                    if strcmp(bools.progress_type,'screw_based_new')
                        for m = 1:3
                            weighted_norm = sqrt(sum(clusters(closest_cluster_nb).mean(1:3,m).^2*L^2 + clusters(closest_cluster_nb).mean(4:6,m).^2));
                            clusters(closest_cluster_nb).mean(:,m) = clusters(closest_cluster_nb).mean(:,m)/weighted_norm;
                        end
                    elseif strcmp(bools.progress_type,'screw_based_old')
                        for m = 1:3
                            weighted_norm = norm(clusters(closest_cluster_nb).mean(1:3,m))*L + norm(clusters(closest_cluster_nb).mean(4:6,m));
                            clusters(closest_cluster_nb).mean(:,m) = clusters(closest_cluster_nb).mean(:,m)/weighted_norm;
                        end
                    elseif strcmp(bools.progress_type,'angle')
                        for m = 1:3 
                            clusters(closest_cluster_nb).mean(1:3,m) = clusters(closest_cluster_nb).mean(1:3,m)/norm(clusters(closest_cluster_nb).mean(1:3,m));
                        end
                    elseif strcmp(bools.progress_type,'arclength')
                        for m = 1:3 
                            clusters(closest_cluster_nb).mean(4:6,m) = clusters(closest_cluster_nb).mean(4:6,m)/norm(clusters(closest_cluster_nb).mean(4:6,m));
                        end
                    elseif strcmp(bools.progress_type,'combined')
                        for m = 1:3                       
                            weighted_norm = sqrt(sum(clusters(closest_cluster_nb).mean(1:3,m).^2*L^2 + clusters(closest_cluster_nb).mean(4:6,m).^2));
                            clusters(closest_cluster_nb).mean(:,m) = clusters(closest_cluster_nb).mean(:,m)/weighted_norm;
                        end
                    end
                end
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
    
    
    %% represent the clusters in a more canonical form for representation purposes;
    descriptor_ref = [1,1,1;0,0,0;0,0,0;1,1,1;0,0,0;0,0,0];
    for k = 1:nb_clusters
        current_cluster_mean = generic_clusters(k).mean;
        A_ = [L*current_cluster_mean(1:3,:),current_cluster_mean(4:6,:)];
        B_ = [descriptor_ref(1:3,:),descriptor_ref(4:6,:)];
        C = A_*B_';
        [U,~,V] = svd(C);
        R = V*U';
        if det(R) < 0
            U(1:3,3) = -U(1:3,3);
            R = V*U';
        end
        current_cluster_mean_rotated(1:3,1:3) = R*current_cluster_mean(1:3,1:3);
        current_cluster_mean_rotated(4:6,1:3) = R*current_cluster_mean(4:6,1:3);
        generic_clusters(k).mean = current_cluster_mean_rotated;
    end
end