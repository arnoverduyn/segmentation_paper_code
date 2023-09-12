function run_segmentation_application(bools,parameters)
% This function runs the application to segment the simulated pouring
% motions. This function consists
% of seven steps:
% 1) Retrieval of the data of the artificial objects
% 2) Initialization of the tuning parameters
% 3) Retrieval of the simulated rigid body trajectory data
% 4) Preprocessing of the input data
% 5) Learning of geometric trajectory-shape primitives from the input data
% 6) Self-supervised segmentation of the simulated trajectories
% 7) Generation of animations

% retrieve data of the designed objects for plotting
[kettle,bottle,table,cup] = retrieve_data_designed_objects();

if bools.print_level
    disp(' ')
    disp('*******************************')
    disp('* Running segmentation script *')
    disp('*******************************')
end

% Initialization of parameters
parameters = initialize_parameters(parameters);
if strcmp(bools.data_type,'real'); parameters.data_size = 6; else; parameters.data_size = 3; end
    
% Retrieve all the data to be segmented (3 times trajectory with kettle + 1 time trajectory with bottle)
if bools.print_level; disp('Retrieve rigid body trajectory data...');end
[pose_trajectories_kettle,pose_trajectories_bottle] = load_all_data(parameters,bools,kettle,bottle,table,cup);

%% Trajectory preprocessing
if bools.print_level; disp(' '); disp('TRAJECTORY PREPROCESSING: ');end
[pose_trajectories_kettle,pose_trajectories_bottle] = trajectory_preprocessing(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,bools,parameters);

%% Learning of geometric trajectory-shape primitives
if bools.print_level; disp(' '); disp('LEARNING OF GEOMETRIC TRAJECTORY-SHAPE PRIMITIVES:'); end
generic_clusters = learning_of_shapes(pose_trajectories_kettle,parameters,bools);

if bools.print_level; disp(' '); disp('LEARNED GEOMETRIC TRAJECTORY-SHAPE PRIMITIVES:'); end
nb_clusters = length(generic_clusters);
for k = 1:nb_clusters
    weighted_cluster = [parameters.L*generic_clusters(k).mean(1:3,:);generic_clusters(k).mean(4:6,:)];
    disp(strcat(['   Primitive ', num2str(k),': ']))
    disp(weighted_cluster)
end

%% Self-supervised segmentation of trajectories
if bools.print_level; disp(' '); disp('SELF-SUPERVISED SEGMENTATION OF TRAJECTORIES:'); end
[pose_trajectories_kettle,pose_trajectories_bottle] = segment_all_trajectories(generic_clusters,pose_trajectories_kettle,pose_trajectories_bottle,parameters,bools);

%% visualize the results
if ~strcmp(bools.data_type,'real') && strcmp(bools.progress_type,'screw_based')
    if bools.bool_plot_all_figures
        plot_segmented_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup)
    end
    if bools.bool_show_intermediate_movies; movie_segmented_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,2,0.4); end

    if bools.bool_create_movie
        close all;
        create_video_segmented_trajectory_kettle(pose_trajectories_kettle,kettle,table,cup)
        create_video_segmented_trajectory_bottle(pose_trajectories_bottle,bottle,table,cup)
    end
end

%% save output of simulation data
if ~strcmp(bools.data_type,'real') && bools.save_simulation_data
    save(strcat('Output/result_kettle_',bools.progress_domain,'_',bools.progress_type,'_',num2str(round(parameters.L*100)),'cm'),'pose_trajectories_kettle')
    save(strcat('Output/result_bottle_',bools.progress_domain,'_',bools.progress_type,'_',num2str(round(parameters.L*100)),'cm'),'pose_trajectories_bottle')
end

%% Evaluate segmentation accuracy based on ground-truth
if strcmp(bools.progress_domain,'geometric') && strcmp(bools.evaluation_domain,'time')
    [pose_trajectories_kettle,~] = revert_segmented_trajectories_to_time_domain(pose_trajectories_kettle,pose_trajectories_bottle,parameters);
end

plot_segmentation_result(pose_trajectories_kettle,parameters,bools)

end