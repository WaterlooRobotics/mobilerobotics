function xcur = Husky(xprev,omegal, omegar, dt)
%% Motion model for Clearpath Robotics Husky UGV
%  xprev = [x,y,theta] Both wheels on each side for Husky are joined throuhg a drive belt
%  therefore, both front and rear wheels on on the same side will have identical rotational velocity.
%  Let these angular velocites be omegal and omegar 
%  Husky's wheel radius is 0.17775m, and distance from the wheels to the cog is 0.2854m
%  as well as the timestep dt. It returns the new robot pose. 


% Motion increment in the body frame
dx_b = dt*[omegal*0.17775+omegar*0.17775 0 (omegal*0.17775*0.2854-omegar*0.17775*0.2854)]';

% Rotation matrix for conversion from body to inertial frame
R = rot(xprev(3),3); 

% Robot state update in inertial frame
xcur = xprev + R*dx_b;

