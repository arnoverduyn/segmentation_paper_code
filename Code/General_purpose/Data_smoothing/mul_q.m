function q = mul_q(q1,q2)
% Multiply two quaternions q1 and q2 to get a third quaternion q3
q = Ql(q1) * q2;
end

function Q = Ql(v )
x = v(1);
y = v(2);
z = v(3);
s = v(4);
Q = [ s -z y x ; z s -x y ; -y x s z ; -x -y -z s];
end
