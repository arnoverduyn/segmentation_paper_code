function twist = calculate_twist_finite_differences(pose_trajectory,parameters)

    T = pose_trajectory;
    N = size(T,3);
    twist = zeros(6,N-1);
    for k = 1:N-1
        % calculate body-fixed twist
        twist_cross = logm_pose(inverse_T(T(:,:,k))*T(:,:,k+1))/parameters.ds;
        twist(1,k) = twist_cross(3,2);
        twist(2,k) = twist_cross(1,3);
        twist(3,k) = twist_cross(2,1);

        twist(1:3,k) = twist(1:3,k);
        twist(4:6,k) = twist_cross(1:3,4);
        
        % change to viewpoint world -> posetwist
        twist(1:3,k) = T(1:3,1:3,k)*twist(1:3,k);
        twist(4:6,k) = T(1:3,1:3,k)*twist(4:6,k);
    end

end