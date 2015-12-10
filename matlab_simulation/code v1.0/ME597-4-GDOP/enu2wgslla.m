% Function [lat,lon,alt] = enu2wgslla(enu, reflat, reflon, refalt)
% returns lat, lon, alt which represents the latitude (degrees),
% longitude (degrees), and altitude above the ellipsoid (in 
% meters) of a point with East, North, Up position
% enu (3 x 1 vector, units in meters) in an ENU coordinate system
% located at latitude reflat (degrees), longitude reflon (degrees)
% and altitude above the WGS84 ellipsoid refalt (meters)
%
% Note: Requires functions enu2wgsxyz.m and wgsxyz2lla.m to be
% in the same directory

function [lat, lon, alt] = wgslla2enu(enu, reflat, reflon, refalt)

xyz = enu2wgsxyz(enu, reflat, reflon, refalt);
[lat, lon, alt] = wgsxyz2lla(xyz);

return