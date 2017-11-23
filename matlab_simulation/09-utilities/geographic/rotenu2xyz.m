%Function xyz=rotenu2xyz(enu (3x1), lat (degrees), lon(degrees))
%rotates a vector from enu reference frame at latitude lat and
%longitude lon into a WGS84 xyz reference frame.  
%
%Note: norm(xyz) and norm(enu) will remain identical
%
%Note: Requires the function rot.m to be in the same directory

function xyz=rotenu2xyz(enu, lat, lon)

R1=rot(90+lon, 3);
R2=rot(90-lat, 1);

R=R2*R1;
xyz = inv(R)*enu;

return;

           