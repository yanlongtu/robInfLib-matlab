function [ qnorm ] = quatnormalize ( q )
% normalize of a quaternion
m = norm(q)
qnorm = q / m;
end

