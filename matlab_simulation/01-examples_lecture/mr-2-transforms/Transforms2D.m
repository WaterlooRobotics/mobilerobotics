%% 2D Transforms
% Performs a few transformations to demonstrate 2D transforms
clc; clear; close all;

% Define the position of an object relative to a robot and an inertial map
% coordinate frame
po_r = [2 2.5]';  % Position of object in robot frame (height z = 0)
pr_m = [4 1]';  % position of robot in map frame
thr_m = [pi/3]; % heading of robot relative to map x axis

% Construct and test 3D rotation matrix to convert from robot to map frame
Rm2r = rot2D(thr_m); % 3D rotation matrix
Rm2rinv = inv(Rm2r); % Inverse of 3D rotation matrix, or Rm_r

Rm2rinv - Rm2r'  % Confirm inverse = transpose for rotation matrix, should return [ 0 0; 0 0]

% Find position of object in map frame using equation on slide 8 of transforms
% p^o_m = (R^r_m)^-1*p^o_r + p^r_m
po_m = Rm2rinv*po_r + pr_m

% Plot axes and locations of objects.
axis_l = 10; % Length of axes to plot

figure(1);clf; hold on;
% Draw Map X-Y axes 
plot( [0 axis_l], [ 0 0], 'k') ; % Map x axis
xlabel('Map x axis')
plot( [0 0], [0 axis_l], 'k'); % Map y axis
ylabel('Map y axis')
text( 1,0.5, 'Map axes');
axis([ -axis_l+2 axis_l+1 -1 axis_l+1])
axis equal
pause(1);

% Draw Robot, Object in Map
drawbot(pr_m(1),pr_m(2),thr_m,0.5,1); % Draw robot
plot(po_m(1), po_m(2), 'go', 'MarkerSize', 6, 'LineWidth', 2); % Draw object
pause(1);

% Draw Robot X-Y axes 
robotaxisep = Rm2rinv*[axis_l 0;0 axis_l]' + [pr_m pr_m]% Robot axis end points
plot( pr_m(1), pr_m(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2) ; % Robot location
plot( [pr_m(1) robotaxisep(1,1)], [pr_m(2) robotaxisep(2,1)], 'r') ; 
plot( [pr_m(1) robotaxisep(1,2)], [pr_m(2) robotaxisep(2,2)], 'r') ; 
text( pr_m(1)+1,pr_m(2)+1, 'Robot axes');
pause(1);

% Object measurement line from robot in map frame
plot([pr_m(1) po_m(1)], [pr_m(2) po_m(2)], 'b'); 





