function xcur = Husky(xprev,omegal, omegar, dt)
%% Motion model for Clearpath Robotics Husky UGV
%  xprev = [x,y,theta] Both wheels on each side for Husky are joined through
%  a drive belt therefore, both front and rear wheels on on the same side 
%  will have identical rotational velocity. Let these angular velocites be 
%  omegal and omegar.  Husky's wheel radius is 0.17775m, and distance from 
%  the wheels to the cog is 0.2854m. The timestep is dt. This function 
%  returns the new robot pose. 

r = 0.1775;
l = 0.2854;

% Motion increment in the body frame -- left thrust (omegal) rotates the
% body clockwise, in the direction of -theta.  Right rotates towards
% positive theta
dx_b = dt*[r*omegal+r*omegar 0 (-r*omegal*l+r*omegar*l)]';

% Rotation matrix for conversion from body to inertial frame -- the
% transpose of the Inertial-to-Body matrix provided by the rot() function.
% The rot() function provides a clockwise rotation.
R = rot(xprev(3),3)'; 

% Robot state update in inertial frame
xcur = xprev + R*dx_b;

