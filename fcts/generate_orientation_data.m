function [q, omega, domega, t] = generate_orientation_data(q1, q2, tau, dt)
%-------------------------------------------------------------------------
% Generate quaternion data, angular velocities and accelerations for DMP
% learning
% Copyright (C) Fares J. Abu-Dakka  2013

    N = round(tau / dt);
    t = linspace(0, tau, N);

    s = struct('s',cell(1),'v',cell(1));
    q = repmat(s,100,1);
    qq = zeros(4,N);
    dqq = zeros(4,N);
    omega = zeros(3,N);
    domega = zeros(3,N);

    % Generate spline data from q1 to q2
    a = minimum_jerk_spline(q1.s, 0, 0, q2.s, 0, 0, tau);
    for i = 1:N
        q(i).s = minimum_jerk(t(i), a);
    end
    for j = 1:3
        a = minimum_jerk_spline(q1.v(j), 0, 0, q2.v(j), 0, 0, tau);
        for i = 1:N
            q(i).v(j,1) = minimum_jerk(t(i), a);
        end
    end

    % Normalize quaternions
    for i = 1:N
        tmp = quat_norm(q(i));
        q(i).s = q(i).s / tmp;
        q(i).v = q(i).v / tmp;
        qq(:,i) = [q(i).s; q(i).v];
    end

    % Calculate derivatives
    for j = 1:4
        dqq(j,:) = gradient(qq(j,:), t);
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
        domega(j,:) = gradient(omega(j,:), t);
    end

    omega(:,1) = [0; 0; 0];
    omega(:,N) = [0; 0; 0];
    domega(:,1) = [0; 0; 0];
    domega(:,N) = [0; 0; 0];
end
