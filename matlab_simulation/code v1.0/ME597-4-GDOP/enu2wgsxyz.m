% Function xyz = enu2wgsxyz(enu, reflat, reflon, refalt) returns
% a 3 x 1 vector xyz which represents the WGS84 XYZ
% coordinates (in meters) of a point with East, North, Up position
% enu (3 x 1 vector, units in meters) in an ENU coordinate system
% located at latitude reflat (degrees), longitude reflon (degrees)
% and altitude above the WGS84 ellipsoid refalt (meters)
%
% Note: Requires functions wgslla2xyz.m and rot.m to be in the 
% same directory

function xyz=enu2wgsxyz(enu, reflat, reflon, refalt)


[m n] = size(enu);
if m ~= 3 | n ~= 1
	error('enu input vector must be 3 x 1');
end

% First, rotate the enu vector to xyz frame

R1=rot(90+reflon, 3);
R2=rot(90-reflat, 1);
R=R2*R1;

diffxyz=inv(R)*enu;

% Then, calculate the xyz of reflat, reflon, refalt

refxyz = wgslla2xyz(reflat, reflon, refalt);

% Add diffxyz to refxyz

xyz = diffxyz + refxyz;

return;

           