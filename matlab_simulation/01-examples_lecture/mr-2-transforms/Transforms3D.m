%% 3D Transforms
% Performs a few transformations to demonstrate 3D transforms using
% rotation matrices
clc; clear; close all;

% Define the position of an object relative to a robot and an inertial map
% coordinate frame
po_r = [2 8 5]';  % Position of object in robot frame (height z = 0)
pr_m = [0 0 0]';  % position of robot in map frame
robot_euler = [pi/4 0 pi/4]; % orientation of robot in euler angles

% Construct and test 3D rotation matrix to convert from robot to map frame
Rr2m = rot(robot_euler(1),1)*rot(robot_euler(2),2)*rot(robot_euler(3),3) % 3D rotation matrix
Rr2minv = inv(Rr2m); % Inverse of 3D rotation matrix, or Rm2r

Rr2minv - Rr2m';  % Confirm inverse = transpose for rotation matrix, should return zero matrix

% Find position of object in map frame using equation on slide 8 of transforms
% p^o_m = (R^r_m)^-1*p^o_r + p^r_m
po_m = Rr2m*po_r + pr_m;

% Plot axes and locations of objects.
axis_l = 10; % Length of axes to plot

figure(1);clf; hold on;
% Draw Map axes 
NED = axis_l*[ 1 0 0; 0 -1 0; 0 0 -1];

plot3( [0 NED(1,1)], [ 0 NED(1,2)], [ 0 NED(1,3)], 'b') ; % Map North axis
xlabel('Map North axis')
plot3( [0 NED(2,1)], [0 NED(2,2)], [0 NED(2,3)], 'b'); % Map East axis
ylabel('Map East axis')
plot3( [0 NED(3,1)], [0 NED(3,2)], [0 NED(3,3)], 'b'); % Map Down axis
zlabel('Map Down axis')
text( 1,0.5, 0, 'Map axes');
grid on;
axis equal;
view(45, 45);
pause(1);

% Draw Robot, Object in Map
load('quad_starmac.mat')
drawquadrotor(qm_fnum, qm_xyz, pr_m(1), pr_m(2), pr_m(3), robot_euler(1), robot_euler(2), robot_euler(3),[0.6 0.1 0.1]);
plot3(po_m(1), po_m(2), po_m(3), 'go', 'MarkerSize', 6, 'LineWidth', 2); % Draw object
pause(1);

% Draw Robot axes 
robotaxisep = Rr2m*NED + [pr_m pr_m pr_m]% Robot axis end points
plot3( [pr_m(1) robotaxisep(1,1)], [pr_m(2) robotaxisep(2,1)],[pr_m(3) robotaxisep(3,1)], 'r') ; 
plot3( [pr_m(1) robotaxisep(1,2)], [pr_m(2) robotaxisep(2,2)],[pr_m(3) robotaxisep(3,2)], 'r') ; 
plot3( [pr_m(1) robotaxisep(1,3)], [pr_m(2) robotaxisep(2,3)],[pr_m(3) robotaxisep(3,3)], 'r') ; 
text( pr_m(1)+1,pr_m(2)+1, pr_m(3)+1, 'Robot axes');
pause(1);

% Object measurement line from robot in map frame
plot3([pr_m(1) po_m(1)], [pr_m(2) po_m(2)], [pr_m(3) po_m(3)], 'b'); 





