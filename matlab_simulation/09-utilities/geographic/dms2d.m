% Function d = dms2d(deg, min, sec) creates a scalar output
% value d which is decimal degrees based on the scalar
% inputs deg, min, sec.
%
% Warning: Results may not be valid unless deg, min, and
% sec are positive

function d=dms2d(deg,min,sec)

[m n] = size(deg);
if m ~= 1 | n ~= 1
	warning('dms2d: deg input value not a scalar');
end

[m n] = size(min);
if m ~= 1 | n ~= 1
	warning('dms2d: min input value not a scalar');
end

[m n] = size(sec);
if m ~= 1 | n ~= 1
	warning('dms2d: sec input value not a scalar');
end

d=deg+min/60+sec/3600;

return;
