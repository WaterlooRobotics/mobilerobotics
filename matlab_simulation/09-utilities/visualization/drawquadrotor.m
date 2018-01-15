 function [ QUAD_HANDLE ] = draw_quadrotor( QUAD_FNUM, QUADXYZ, CX, CY, CZ, PHI, THETA, PSI, FCOLOR )
%DRAW_QUADROTOR draws a quadrotor oriented and translated 
%               in the current figure. The quadrotor is stored in trisurf
%               format, which requires the number of faces, node positions
%               and center location transformation as inputs.  An example
%               quadrotor model has been stored in quad_starmac.mat, which
%               can be loaded with load('quad_starmac.mat').
%   Parameters
%   Input, integer QUAD_FNUM, # of faces in quadrotor model
%   Input, real QUADXYZ(NODENUM,3), quadrotor xyz node positions (m)
%   Input, real PHI, current pitch (rad)
%   Input, real THETA, current roll (rad)
%   Input, real PSI, current yaw (rad)
%   Input, real CX, current center x position (m)
%   Input, real CY, current center y position (m)
%   Input, real CZ, current center z position (m)
%   Input, string or [r g b] FCOLOR, color for the faces of the model
%   Output, handle QUAD_HANDLE, patch handle for quadrotor

%first need to get euler rotation matrix for the rotor
R = [rot(PHI,1)*rot(THETA,2)*rot(PSI,3)];  %rotation matrix
%R = SpinCalc('EA123toDCM', radtodeg([PHI, THETA, PSI]));
T = zeros(length(QUADXYZ(:,1)), 3); 
T(:,1) = CX; T(:,2) = CY; T(:,3) = CZ;  %translation matrix

%perform rotation and translation
xyz = R'*QUADXYZ' + T';   %rotate and translate from robot frame to inertial

%draw quadrotor model
QUAD_HANDLE = trisurf(QUAD_FNUM, ...
                      xyz(1,:), xyz(2,:), xyz(3,:), ...
                      'FaceColor', FCOLOR,'EdgeColor',FCOLOR);

end

