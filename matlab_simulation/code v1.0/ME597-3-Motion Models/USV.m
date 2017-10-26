function xcur = USV(xprev,Tl, Tr, m, ll, lr, dt)
%% Motion model for a 2 thruster USV, Such as Clearpath Robotics Heron Platform 
%  xprev = [x,y,theta,vx,vy,vtheta]'right thrust and left thrust are denoted as Tl and Tr, mass of m,
%  perpendicular distance from thruster to cog is ll and lr for right thruster and left thruster respectivly
%  as well as the timestep dt. It returns the new robot pose. 

%Very rough drag calculated for Clearpath's Heron, comment this out and remove from
%velocity equations if using a diffrent USV, or sub in your own vehicles
%properties.Note drag for theta was not modeled, so rotational speed can
%get very large
%A~0.31364m^2, assuming drag coeffecient of ~0.7

F_drag=-0.5*1000*0.7*0.31364*xprev(4)^2;

if xprev(4)<0
    F_drag=-F_drag;
end

%update velocities
v_x=xprev(4) + ((Tl/m+Tr/m+F_drag/m)*dt);
v_theta=xprev(6) + ((Tl*ll-Tr*lr)/m)*dt;

% Motion increment in the body frame
dx_b = [v_x*dt 0 v_theta*dt]';

% Rotation matrix for conversion from body to inertial frame
R = rot(xprev(3),3); 

% Robot state update in inertial frame
xcur = [xprev(1:3) + R*dx_b; v_x;0;v_theta];


