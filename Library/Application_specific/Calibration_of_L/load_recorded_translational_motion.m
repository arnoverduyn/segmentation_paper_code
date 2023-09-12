function [T,N] = load_recorded_translational_motion()
    raw_data = csvread('Data/translational_motions/Trial1_translation.csv');
    time_vector = raw_data(:,1);
    time_vector = time_vector - time_vector(1);
    pos = raw_data(:,2:4)';
    quat = raw_data(:,5:8);
    N = length(time_vector);
    T = zeros(4,4,N);
    for k = 1:N
        T(1:3,1:3,k) = quat2rotm(quat(k,:));
        T(1:3,4,k) = pos(:,k);
        T(4,4,k) = 1;
    end
    dt = 0.02;
    time_vector_new = 0:dt:time_vector(end);
    T = interpT(time_vector,T,time_vector_new);
end