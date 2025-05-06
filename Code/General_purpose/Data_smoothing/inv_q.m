function q_new = inv_q(q)
% Convert a quaternion to its inverse

q_new = zeros(4,1);
q_new(1:3) = -q(1:3);
q_new(4) = q(4);
q_new = q_new/norm(q);
end