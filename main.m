close all; clc; clear variables;

restoredefaultpath

addpath(genpath('Data'))
addpath(genpath('Code'))

%%
% This script runs the proposed self-supervised segmentation algorithm to
% segment temporal rigid-body trajectories using a novel screw-based
% trajectory descriptor

%% Setup of the experiments
bools.data_type = 'simulation'; % {'simulation', 'new_simulation', 'real'}
bools.evaluation_domain = 'geometric'; % {'time','geometric'} -> domain in which the segmentation performance is evaluated
bools.bool_compare_all_results = 0; % {0,1} -> if 1, the saved outputs of the different methods discussed in the paper is regenerated
                                    %       -> if 0, the performance of the descriptor type and metric specified below will be evaluated

%% Setup of the descriptor type and metric
bools.progress_domain = 'geometric'; % 'time' or 'geometric'

bools.progress_type = 'screw_based_new';    
% 'time'
% 'angle'           -> conform Roth2005
% 'arclength'       -> conform Yang2023
% 'combined'        -> conform Park1995
% 'screw_based_old' -> conform Joris2010
% 'screw_based_new' -> proposed method

parameters.ds = 0.02; 
% expressed in [s] for timebased  (value in paper 0.05)
% expressed in [rad] for geometric based on angle (value in paper 0.0524  -> 3 degrees)
% expressed in [m] for geometric  (values in paper: 0.02 for screw_based_new, screw_based_old, and arclength)

parameters.L = 0.3; % [m] predefined value (0.3 for 'screw based', 'combined' or 'angle', 0 for 'arclength')

%% Setup of the hyperparameters of the segmentation algorithm
parameters.alpha = 0.80; % [-] percentage that has to be represented as a translation

parameters.min_Q = 0.1; 
% process covariance of the clusters
% expressed in [m] for geometric   (value in paper 0.1 for screw_based_new, screw_based_old, and arclength) 
%                                  (value in paper 0.05 for arclength while neglecting object's rotation)
% expressed in [m/s] for timebased (value in paper 0.1)

parameters.outlier_percentage = 5; % [%] 5 for geometric screw based, 5 for geometric arclength , 2 for temporal

%% Setup of the generated outputs
bools.print_level = 1; % 1 -> show output in command window
                       % 0 -> show no output in command window
bools.bool_plot_all_figures = 1;

bools.bool_show_intermediate_movies = 0;
% Above intermediate movies includes visualizations of 
%  (1) the input data
%  (2) visualization of the generated segments

%% Save outputs
bools.save_simulation_data = 0; % 1 -> save an exact copy of the simulation data (for reproducability reasons)
bools.save_output = 0; % 1 -> save the generated output (evaluation of segmentation performance)
bools.bool_create_movie = 0; % create and save final movie
 
%% run the segmentation application
if strcmp(bools.data_type,'simulation') && bools.bool_compare_all_results 
    plot_comparison(); % visualize previously stored outputs
else
    run_segmentation_application(bools,parameters) % generate new outputs
end
    
    