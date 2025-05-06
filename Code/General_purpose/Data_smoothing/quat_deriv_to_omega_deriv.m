function [omega,omegadot] = quat_deriv_to_omega_deriv(q,qdot,qddot)
% Convert quaternion and derivatives to angular velocities and derivatives

omega = zeros(length(q),3);
omegadot = zeros(length(q),3);

for i=1:length(q)
    
    %Quaternions And Dynamics - Basil Graf
    
    omegaq= mul_q(2*qdot(i,:) , inv_q(q(i,:))); % formula qdot = 1/2*omega*q    =>  omega = 2*qdot*q^-1
    omega(i,:) = omegaq(1:3);
    
    omegadotq = mul_q((2*qddot(i,:)' - mul_q(omegaq,qdot(i,:)')) , inv_q(q(i,:)));
    omegadot(i,:) = omegadotq(1:3);

end
