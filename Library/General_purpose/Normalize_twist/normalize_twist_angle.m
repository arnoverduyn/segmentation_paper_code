function twist = normalize_twist_angle(twist)

    N = size(twist,2);
    for k = 1:N % normalize geometric twists
        twist(1:3,k) = twist(1:3,k)/norm(twist(1:3,k)); % normalization
    end
end