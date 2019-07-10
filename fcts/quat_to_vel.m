function [ omega, domega ] = quat_to_vel(q, dt, tau)
% calculate angular vel/acc using quaternions

N=tau/dt;

for i = 1:N
    qq(:,i) = [q(i).s; q(i).v];
end

% Calculate derivatives
for j = 1:4
    dqq(j,:) = gradient(qq(j,:))/dt;
end

% Calculate omega and domega
for i = 1:N
    dq.s = dqq(1,i);
    for j = 1:3
        dq.v(j,1) = dqq(j+1,i);
    end
    omega_q = quat_mult(dq, quat_conjugate(q(i)));
    omega(:,i) = 2*omega_q.v;
end
for j = 1:3
    domega(j,:) = gradient(omega(j,:))/dt;
end


end

