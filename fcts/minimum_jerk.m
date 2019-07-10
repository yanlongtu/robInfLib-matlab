% Returns the value and derivative of the minimum jerk spline with coefficients a
% at time t
% Copyright (C) Fares J. Abu-Dakka  2013

function [pos, vel, acc] = minimum_jerk(t, a)
t2 = t * t;
t3 = t2 * t;
t4 = t2 * t2;
t5 = t3 * t2;
pos = a(6) * t5 + a(5) * t4 + a(4) * t3 + a(3) * t2 + a(2) * t + a(1);
vel = 5 * a(6) * t4 + 4 * a(5) * t3 + 3 * a(4) * t2 + 2 * a(3) * t + a(2);
acc = 20 * a(6) * t3 + 12 * a(5) * t2 + 6 * a(4) * t + 2 * a(3);
end
