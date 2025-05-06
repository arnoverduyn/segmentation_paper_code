function twist = normalize_screwbased_Joris_twist(twist,parameters)

    N = size(twist,2);
    for k = 1:N % normalize geometric twists
        weighted_norm = parameters.L*norm(twist(1:3,k)) + norm(twist(4:6,k));
        twist(:,k) = twist(:,k)/weighted_norm;
    end

end