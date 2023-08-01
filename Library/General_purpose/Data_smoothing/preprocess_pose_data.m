function [smoothed_pose_coordinates,twist,tdot] = preprocess_pose_data(measured_pose_coordinates,params_preprocessing)
% Preprocess the given measured pose data to obtain smoothed pose coordinates and twist
%
% Input: measured_pose_coordinates = given pose coordinates with quaternion for orientation structed as [x,y,z,qx,qy,qz,qw]
% Input: params_preprocessing = parameters for Kalman smoother

% Apply Kalman smoother
[data_smooth,datadot_smooth,datadotdot_smooth] = kalman_smoother(measured_pose_coordinates,params_preprocessing);

%% Load in translation + derivatives in results
smoothed_pose_coordinates(:,1:3) =  data_smooth(:,1:3);
twist(:,4:6) = datadot_smooth(:,1:3);
tdot(:,4:6) = datadotdot_smooth(:,1:3);

%% Load in rotation + derivatives in results
q = data_smooth(:,4:7);
for i=1:size(q,1)
    q(i,:) = q(i,:)/norm(q(i,:)); % normalize orientation
end
qdot = datadot_smooth(:,4:7);
qddot = datadotdot_smooth(:,4:7);

% Convert quaternion and derivatives to angular velocities and derivatives
[omega,omegadot] = quat_deriv_to_omega_deriv(q,qdot,qddot);
smoothed_pose_coordinates(:,4:7) =  q;
twist(:,1:3) = omega;
tdot(:,1:3) = omegadot;
end