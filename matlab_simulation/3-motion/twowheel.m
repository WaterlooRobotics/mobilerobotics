function xcur = twowheel(xprev,phidl,phidr,r,l,dt)
%% Two wheel robot model kinematics. This function takes in previous state 
%  xprev = [x,y,theta]', left wheel and right wheel rotation rates phidl
%  and phidr, the wheel radius r and the distance from wheel to cg l, as
%  well as the timestep dt. It returns the new robot pose. 


% Motion increment in the body frame
dx_b = dt*[r*(phidl + phidr)/2 0 r*(phidr - phidl)/(2*l)]';

% Rotation matrix for conversion from body to inertial frame
R = rot(xprev(3),3); 

% Robot state update in inertial frame
xcur = xprev + R*dx_b;

