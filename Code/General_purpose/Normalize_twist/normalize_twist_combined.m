function twist = normalize_twist_combined(twist, parameters)

    N = size(twist,2);
    for k = 1:N % normalize geometric twists
        weighted_norm = sqrt(dot(twist(1:3,k),twist(1:3,k))*parameters.L^2 + dot(twist(4:6,k),twist(4:6,k)));
        twist(:,k) = twist(:,k)/weighted_norm; % normalization
    end
end