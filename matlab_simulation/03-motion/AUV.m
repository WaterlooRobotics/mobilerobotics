function xcur = AUV(xprev,Tx, Ty, Tz, m,dt)
%% AUV motion model with 3 thrusters and constant attitude. This function takes in previous state 
%  xprev = [x,y,z,vx,vy,vz]' thrust values in Newtons, Tx,Ty,Tz, mass, m,
%  as well as the timestep dt. It returns the new robot pose. 

%Rough drag calculation, assuming a smooth sphere with radius 0.25m
%Coeffecient of drag ~0.47, area=0.2m^2

F_drag_x=-0.5*1000*0.47*0.2*xprev(4)^2;
F_drag_y=-0.5*1000*0.47*0.2*xprev(5)^2;
F_drag_z=-0.5*1000*0.47*0.2*xprev(6)^2;

%change the direction of drag to be against direction of travel
if xprev(4)<0
    F_drag_x=-F_drag_x;
end

if xprev(5)<0
    F_drag_y=-F_drag_y;
end

if xprev(6)<0
    F_drag_z=-F_drag_z;
end

%Update velocites
vx=xprev(4)+ (Tx/m+F_drag_x/m)*dt;
vy=xprev(5)+ (Ty/m+F_drag_y/m)*dt;
vz=xprev(6)+ (Tz/m+F_drag_z/m)*dt;

% Motion increment in the body frame, where v=(T/m)*dt
dx_b = dt*[vx vy vz]';

%No rotation required

% Robot state update in inertial frame
xcur = [xprev(1:3) + dx_b;vx;vy;vz];

