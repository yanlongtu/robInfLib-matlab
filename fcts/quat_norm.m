function [ qnorm ] = quat_norm ( q )
% calculate the norm of a quaternion
a=q.s;
b=q.v;
qnorm=norm([a b']);

end

