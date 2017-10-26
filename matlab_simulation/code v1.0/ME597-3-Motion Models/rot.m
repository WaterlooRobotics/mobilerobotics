%Function R=rot(angle (degrees), axis) returns a 3x3
%rotation matrix for rotating a vector about a single
%axis.  Setting axis = 1 rotates about the e1 axis,
%axis = 2 rotates about the e2 axis, axis = 3 rotates
%about the e3 axis.

function R=rot(angle, axis)
%function R=rot(angle (degrees), axis) 

R=eye(3);
cang=cos(angle*pi/180);
sang=sin(angle*pi/180);

if (axis==1)
R(2,2)=cang;
R(3,3)=cang;
R(2,3)=sang;
R(3,2)=-sang;
end;

if (axis==2)
R(1,1)=cang;
R(3,3)=cang;
R(1,3)=-sang;
R(3,1)=sang;
end;

if (axis==3)
R(1,1)=cang;
R(2,2)=cang;
R(2,1)=-sang;
R(1,2)=sang;
end;

return;
