function run_segmentation_application(bools,parameters)
% This function runs the application to segment the simulated pouring
% motions into generic geometric trajectory shapes. This function consists
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

% Set weighting factor L 
if bools.bool_calibrate_L
    if bools.print_level; disp('Calibrate weighting factor L ...'); end
    parameters.L = calibrate_L(parameters,bools,kettle);
end

% Retrieve all the data to be segmented (3 times trajectory with kettle + 1 time trajectory with bottle)
if bools.print_level; disp('Retrieve rigid body trajectory data...');end
[pose_trajectories_kettle,pose_trajectories_bottle] = generate_simulation_data_trajectories(parameters);

if bools.bool_plot_all_figures
    plot_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup)
end
if bools.bool_show_intermediate_movies; movie_input_trajectories(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,5,0.1); end

%% Trajectory preprocessing
if bools.print_level; disp(' '); disp('TRAJECTORY PREPROCESSING: ');end
[pose_trajectories_kettle,pose_trajectories_bottle] = trajectory_preprocessing(pose_trajectories_kettle,pose_trajectories_bottle,kettle,bottle,table,cup,bools,parameters);

%% Learning of geometric trajectory-shape primitives
if bools.print_level; disp(' '); disp('LEARNING OF GEOMETRIC TRAJECTORY-SHAPE PRIMITIVES:'); end
generic_clusters = learning_of_shapes(pose_trajectories_kettle,parameters,bools);

%% Self-supervised segmentation of trajectories
if bools.print_level; disp(' '); disp('SELF-SUPERVISED SEGMENTATION OF TRAJECTORIES:'); end
for data = 1:3
    input_trajectory = pose_trajectories_kettle(data);
    segmented_trajectory = segment_trajectory(generic_clusters,input_trajectory,bools,parameters);
    pose_trajectories_kettle(data).associated_cluster_indices = segmented_trajectory.associated_cluster_indices;
    pose_trajectories_kettle(data).segments = segmented_trajectory.segments;
end
input_trajectory = pose_trajectories_bottle;
segmented_trajectory = segment_trajectory(generic_clusters,input_trajectory,bools,parameters);
pose_trajectories_bottle.associated_cluster_indices = segmented_trajectory.associated_cluster_indices;
pose_trajectories_bottle.segments = segmented_trajectory.segments;

%% visualize the results
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