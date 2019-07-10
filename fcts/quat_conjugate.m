function qc = quat_conjugate(q)
%-------------------------------------------------------------------------
% Quaternion conjugation
% Copyright (C) Fares J. Abu-Dakka  2013

qc.s = q.s;
qc.v = -q.v;
end
