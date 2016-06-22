function xcur = USV(xprev,Tl, Tr, m, ll, lr, dt)
%% Motion model for a 2 thruster USV, Such as Clearpath Robotics Heron Platform 
%  xprev = [x,y,theta]'right thrust and left thrust are denoted as Tl and Tr, mass of m,
%  perpendicular distance from thruster to cog is ll and lr for right thruster and left thruster respectivly
%  as well as the timestep dt. It returns the new robot pose. 


% Motion increment in the body frame
dx_b = dt*[Tl/m+Tr/m 0 (Tl*ll-Tr*lr)/m]';

% Rotation matrix for conversion from body to inertial frame
R = rot(xprev(3),3); 

% Robot state update in inertial frame
xcur = xprev + R*dx_b;

