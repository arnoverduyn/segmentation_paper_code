function [F,Q,H,R] = system_and_measurementModel_kalman6(kalman_model_parameters,trajectory_type)
%Construct system model equations and measurement model equations for Kalman Filter/smoother constant djerk model
%
% Input: kalman_model_parameters = parameters % TODO break this up again into separate parameters
% 
% Output: F = system equations, 5x5 matrix
%         Q = process error covariance, 5x5 matrix
%         H = measurement equations, 1x5 matrix
%         R = measurement error covariance, 5x5 matrix

%Loading parameters
sigma_p = kalman_model_parameters.init_system_noise_cov;
if strcmp(trajectory_type,'pos')
    sigma_m = kalman_model_parameters.init_meas_noise_cov_pos;
else
    sigma_m = kalman_model_parameters.init_meas_noise_cov_quat;
end
dT = kalman_model_parameters.time_step;

%System matrix (this is basically a Taylor series expansion of the position (first row), velocity (second row), etc.
F=[[1 0 0]' [dT 1 0]' [dT^2/2 dT 1]'];

%Process disturbance factor (final term of Taylor series expansion)
q=[dT^3/6 dT^2/2 dT];
qq=q'*q;
Q=(sigma_p)^2*(qq);

%Measurement system
H=[1 0 0; 0 1 0]; % position measurement + virtual velocity measurement

%Noise covariance matrix
R=[sigma_m^2, 0; 0, 0.1*sigma_m^2/dT^2];