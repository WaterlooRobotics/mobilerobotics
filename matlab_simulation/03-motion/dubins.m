function xcur = dubins(xprev,v,omega,dt)
%% Dubins robot model kinematics. This function takes in previous state 
%  xprev = [x,y,theta]', speed v and angular rate omega commands as
%  well as the timestep dt. It returns the new robot pose.  


% Motion increment in the body frame
dx_b = dt*[v 0 omega]';

% Rotation matrix for conversion from inertial frame to the robot frame.
R = rot(xprev(3),3); 

% Robot state update in inertial frame -- use transpose of R
xcur = xprev + R'*dx_b;
