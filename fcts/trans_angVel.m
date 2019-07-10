function [ localVel ] = trans_angVel(qDes, wDes, dt, q0)
% transform desired angular velocity into the Euclidean space

q_new=quat_mult(quat_exp(wDes/2*dt), qDes); % desired quaternion at next time step

zeta1=quat_log(qDes, q0);
zeta2=quat_log(q_new,q0);

localVel=(zeta2-zeta1)/dt;

end

