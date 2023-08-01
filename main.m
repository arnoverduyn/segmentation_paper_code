close all; clc; clear variables;

load_library();
load_data();

%%
% This script runs the proposed self-supervised segmentation algorithm to
% segment temporal rigid-body trajectories into generic geometric
% trajectory-shape primitives

% Booleans (to be set by the user)
bools.bool_calibrate_L = 1;  % If zero, a predefined value for L will be set
parameters.L = 0.3; % [m] predefined value
parameters.alpha = 0.97; % [-] percentage that has to be represented as a translation

parameters.ds = 0.02; % [m] 0.02
parameters.min_Q = 0.1; % [m] process covariance of the clusters
parameters.outlier_percentage = 5; % [%]

bools.order_learning_data = 0; % {0,1} Order data based on some heuristic before incremental clustering.
                               % This is only possible for offline learning applications.

bools.print_level = 1; % 1 -> show output in command window
                       % 0 -> show no output in command window
bools.bool_plot_all_figures = 1;

bools.bool_show_intermediate_movies = 0;
% Above intermediate movies includes visualizations of 
%  (1) the input data
%  (2) the smoothed data
%  (3) calculated instantaneous reference point
%  (4) visualization of the generated segments

bools.bool_create_movie = 0; % create and save final movie
 
% run the segmentation application
run_segmentation_application(bools,parameters)