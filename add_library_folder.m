function add_library_folder()
% This function adds the necessary library folders to the matlab path

addpath('Library/General_purpose/Robotics_functions')
addpath('Library/General_purpose/Data_smoothing')
addpath('Library/General_purpose/Proposed_progress_rate')
addpath('Library/General_purpose/Normalize_twist')

addpath('Library/Application_specific')
addpath('Library/Application_specific/Calibration_of_L')
addpath('Library/Application_specific/Data_retrieval')
addpath('Library/Application_specific/Plotting_functions')
addpath('Library/Application_specific/Segmentation_application')
addpath('Library/Application_specific/Segmentation_application/Trajectory_preprocessing')
addpath('Library/Application_specific/Segmentation_application/Template_learning')
addpath('Library/Application_specific/Segmentation_application/Trajectory_segmentation')
end
