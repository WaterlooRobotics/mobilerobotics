% Function enu = wgsxyz2enu(xyz, reflat, reflon, refalt) returns
% a 3 x 1 vector enu which represents the East, North, and Up
% coordinates (in meters) of a point with WGS84 xyz coordinates
% xyz (3 x 1 vector, units in meters) in an ENU coordinate system
% located at latitude reflat (degrees), longitude reflon (degrees)
% and altitude above the WGS84 ellipsoid refalt (meters)
%
% Note: Requires functions wgslla2xyz.m and rot.m to be in the 
% same directory

function enu=wgsxyz2enu(xyz, reflat, reflon, refalt)

[m n] = size(xyz);
if m ~= 3 | n ~= 1
	error('wgsxyz2enu: xyz input vector must be 3 x 1');
end

[m n] = size(reflat);
if m ~= 1 | n ~= 1
	error('wgsxyz2enu: reflat input vector must be scalar');
end

[m n] = size(reflon);
if m ~= 1 | n ~= 1
	error('wgsxyz2enu: reflon input vector must be scalar');
end

[m n] = size(refalt);
if m ~= 1 | n ~= 1
	error('wgsxyz2enu: refalt input vector must be scalar');
end

% First, calculate the xyz of reflat, reflon, refalt

refxyz = wgslla2xyz(reflat, reflon, refalt);

% Difference xyz from reference point

diffxyz = xyz - refxyz;

% Now rotate the (often short) diffxyz vector to enu frame

R1=rot(90+reflon, 3);
R2=rot(90-reflat, 1);
R=R2*R1;

enu=R*diffxyz;

return;

           