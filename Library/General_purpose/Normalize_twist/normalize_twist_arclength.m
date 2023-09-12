function twist = normalize_twist_arclength(twist)

    N = size(twist,2);
    for k = 1:N % normalize geometric twists
        twist(4:6,k) = twist(4:6,k)/norm(twist(4:6,k)); % normalization
    end
end