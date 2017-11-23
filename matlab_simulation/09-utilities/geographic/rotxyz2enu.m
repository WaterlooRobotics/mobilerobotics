%Function enu=rotxyz2enu(xyz (3x1), lat (degrees), lon(degrees))
%rotates a vector from the WGS84 xyz reference frame into an ENU
%reference frame at latitude lat and longitude lon
%
%Note: norm(xyz) and norm(enu) will remain identical
%
%Note: Requires the function rot.m to be in the same directory

function enu=rotxyz2enu(xyz, lat, lon)

R1=rot(90+lon, 3);
R2=rot(90-lat, 1);

R=R2*R1;
enu=R*xyz;

return;

           