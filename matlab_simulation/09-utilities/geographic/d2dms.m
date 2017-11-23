% Function [deg, min, sec] = d2dms(degrees) returns a 3 scalars
% which are the scalar decimal values degrees represented
% in degrees, arc-minutes, and arc-seconds
%
% Note: If degrees is negative, then deg, min, sec will all
% be negative

function [d, min, sec] = d2dms(deg)

[m n] = size(deg);
if m ~= 1 | n ~= 1
	warning('d2dms: input degrees not a scalar');
end

d = fix(deg);
min = fix(rem(deg*60,60));
sec = rem(deg*3600,60);

return
