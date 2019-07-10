%  Calculates the coefficients of the minimum jerk spline with given value,
%  first, and second derivative at the end points (0 and T)
% Copyright (C) Fares J. Abu-Dakka  2013

function a = minimum_jerk_spline(x0, dx0, ddx0, x1, dx1, ddx1, T)
    a(1) = x0;
    a(2) = dx0;
    a(3) = ddx0 / 2;
    a(4) = (20 * x1 - 20 * x0 - (8 * dx1 + 12 * dx0) * T - ...
        (3 * ddx0 - ddx1) * T * T ) / (2 * T * T * T);
    a(5) = (30 * x0 - 30 * x1 + (14 * dx1 + 16 * dx0) * T + ...
        (3 * ddx0 - 2 * ddx1) * T * T ) / (2 * T * T * T * T);
    a(6) = (12 * x1 - 12 * x0 - (6 * dx1 + 6 * dx0) * T - ...
        (ddx0 - ddx1) * T * T ) / (2 * T * T * T * T * T);
end
