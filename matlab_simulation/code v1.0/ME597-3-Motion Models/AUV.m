function xcur = AUV(xprev,Tx, Ty, Tz, m,dt)
%% AUV motion model with 3 thrusters and constant attitude. This function takes in previous state 
%  xprev = [x,y,z]' thrust values in Newtons, Tx,Ty,Tz, mass, m,
%  as well as the timestep dt. It returns the new robot pose. 


% Motion increment in the body frame, where v=(T/m)*dt
dx_b = dt*[Tx/m Ty/m Tz/m]';

%No rotation required

% Robot state update in inertial frame
xcur = xprev + dx_b;

