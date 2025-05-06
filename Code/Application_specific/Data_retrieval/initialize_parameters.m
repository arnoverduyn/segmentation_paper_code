function parameters = initialize_parameters(parameters)
% This function sets internal parameters for:
% 1) The generation of synthetic data
% 2) The covariance matrices for the Kalman Smoother
    
    % Sample frequency of signals
    parameters.f_sample = 50; % Hz
    parameters.dt = 1/parameters.f_sample;
    
    % noise level on the synthetic data (Additive white noise)
    parameters.std_dev_angle = 1/sqrt(3)*2/180*pi; % standard deviation of 2 degrees 
    parameters.std_dev_pos = 1/sqrt(3)*0.001; % standard deviation of 2 mm
    
    % Smooth calibration data with a Kalman smoother
    Q = 10^(3); % process covariance matrix (std dev on jerk)
    R_pos = 0.005; % measurement covariance matrix for position
    R_quat = 0.05; % measurement covariance matrix for quaternion
    parameters.kalman.model_parameters = ...
        struct('init_error_cov',[1 10 100], ... % diagonal of initial state covariance matrix, not very important
        'init_system_noise_cov', Q, ... % process noise [m/s^5], this parameter is typically tuned by the user
        'init_meas_noise_cov_pos',R_pos, ... % measurement noise [m], typically known from the used sensor
        'init_meas_noise_cov_quat',R_quat, ... % measurement noise [m], typically known from the used sensor
        'time_step',parameters.dt); % time step [s]
end