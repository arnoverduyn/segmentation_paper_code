function [T,N] = load_recorded_pouring_motion(number)
    if number == 1
        raw_data = csvread('Data/pouring_motions/Trial1_coffee_kettle_ref_top.csv');
    elseif number == 2
        raw_data = csvread('Data/pouring_motions/Trial2_coffee_kettle_ref_top.csv');
    elseif number == 3
        raw_data = csvread('Data/pouring_motions/Trial3_coffee_kettle_ref_top.csv');
    elseif number == 4
        raw_data = csvread('Data/pouring_motions/Trial1_coffee_kettle_ref_CoM.csv');
    elseif number == 5
        raw_data = csvread('Data/pouring_motions/Trial2_coffee_kettle_ref_CoM.csv');
    elseif number == 6
        raw_data = csvread('Data/pouring_motions/Trial3_coffee_kettle_ref_CoM.csv');
    end
    time_vector = raw_data(:,1);
    time_vector = time_vector - time_vector(1);
    pos = raw_data(:,2:4)';
    quat = [raw_data(:,8),raw_data(:,5:7),];
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
    N = size(T,3);
        
end