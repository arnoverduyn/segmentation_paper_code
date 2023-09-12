function [P,Pdot,Pddot] = kalman_smoother(data,params_preprocessing)
% Smooth the data using a Kalman smoother with a constant DJerk model
% We consider the controls as part of the state, sometimes referred to as an augmented Kalman filter/smoother
% The Kalman smoother eliminates lag in the Kalman Filter estimation by doing a backward pass over the states
%
% Input: data = NxM input data with N the samples and M the different coordinates
%        params_preprocessing = parameters %TODO explain, maybe break this structure up into separated parameter values
% Output: P = smoothed data
%         Pdot = smoothed derivative of data
%         Pddot = smoothed second derivative of data
%         Pdddot = smoothed third derivative of data

[N,M] = size(data);

% Initialize results structure
data_smooth = zeros(N,M);
data_filtered = zeros(N,M);
datadot_smooth = zeros(N,M);
datadot_filtered = zeros(N,M);
datadotdot_smooth = zeros(N,M);
datadotdot_filtered = zeros(N,M);

% For all coordinates in the data
for k=1:M
    
    if k <= 3 % position coordinates
        % Build system equations, measurement equations, process error covariance matrix, measurement error covariance matrix
        [F,Q,H,R] = system_and_measurementModel_kalman(params_preprocessing.kalman.model_parameters,'pos');
    else % quaternions
        % Build system equations, measurement equations, process error covariance matrix, measurement error covariance matrix
        [F,Q,H,R] = system_and_measurementModel_kalman(params_preprocessing.kalman.model_parameters,'quat');
    end
    init_cov_param = params_preprocessing.kalman.model_parameters.init_error_cov; % initial state covariance error

    % Five states: position, velocity, acceleration, jerk, djerk
    X_FILT = zeros(3,N); 
    X_PRED = zeros(3,N);
    P_FILT = cell(3,3,N);
    P_PRED = cell(3,3,N);
    
    z = data(:,k)'; % measured positions
    
    % Initial state and initial state covariance matrix
    x_old = [z(1) 0 0]';
    P_old = diag(init_cov_param);
    
    % Kalman filter iteration (forward pass)
    for j=1:N %for each timestep
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Kalman predict: Prediction of current state using old state
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x_pred = F*x_old; %predict state
        P_pred = F*P_old*F' + Q; %prediction error covariance
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Kalman correct: correction of current state using predicted state
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        nu = [z(:,j);0] - H*x_pred; %innovation
        
        S = H*P_pred*H' + R; %innovation covariance
        K = P_pred*H'/S; %kalman gain (K = P_pred*H'*S^(-1))
        x_filt = x_pred + K*nu; %corrected state
        P_filt = (speye(3,3)-K*H)*P_pred; %corrected state covariance, Pfilt = Ppred - K S K'
        
        % Store result of this iteration
        X_PRED(:,j)= x_pred;
        X_FILT(:,j)= x_filt;
        P_PRED{j} = P_pred;
        P_FILT{j} = P_filt;
        data_filtered(j,k) = x_filt(1);
        datadot_filtered(j,k) = x_filt(2);
        datadotdot_filtered(j,k) = x_filt(3);
        
        % Initialize next step
        x_old = x_filt;
        P_old = P_filt;
    end
    
    % Initialize smoothing vector
    data_smooth(N,k) = x_filt(1);
    datadot_smooth(N,k) = x_filt(2);
    datadotdot_smooth(N,k) = x_filt(3);
    
    % Start from last state
    x_new = X_FILT(:,N);
    P_new = P_FILT{N};
    
    % Kalman Smoother iteration (backward pass)
    for j=N-1:-1:1
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Kalman smooth (Raugh-Tung-Striebel)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x_filt = X_FILT(:,j);
        P_filt = P_FILT{j};
        x_pred = X_PRED(:,j+1);
        P_pred = P_PRED{j+1};
        
        J = P_filt*F'*(P_pred)^(-1); 
        x_smooth = x_filt + J*(x_new - x_pred);
        P_smooth = P_filt + J*(P_new - P_pred)*J';

%         J = P_filt*F'/P_FILT{j+1}; 
%         x_smooth = x_filt + J*(x_new - X_FILT(:,j+1));
%         P_smooth = P_filt + J*(P_new - P_FILT{j+1})*J';
        
        data_smooth(j,k) = x_smooth(1);
        datadot_smooth(j,k) = x_smooth(2);
        datadotdot_smooth(j,k) = x_smooth(3);
        
        % Next step
        x_new = x_smooth;
        P_new = P_smooth;
    end
end

P = data_smooth;
Pdot = datadot_smooth;
Pddot = datadotdot_smooth;
%data_filtered %extra output after Kalman Filter step without smoothing

end