function [ q ] = quat_exp( w )
% transform a 3-D angular velociy to 4-D quaternion 
tmp = norm(w);
if tmp >1e-12
         q_w = [ cos(tmp) ; sin(tmp) * w/tmp ];
else
         q_w = [1; 0; 0; 0];
end
dq = q_w/norm(q_w);
 
q.s=dq(1);
q.v=dq(2:4);

end

