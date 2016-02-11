%% Spincalc demo
% Demonstrates conversions between 321-Euler angles, quaternions, twist
% parameters (Euler vectors and angle), and direction cosine matrices, and
% moves through gimbal lock where Euler angle definition fails.  Spincalc
% itself can do more, see help SpinCalc
clc; clear; close all;

T = 100; % Timesteps
r = 2; % Radius of circle flown

% Define the position of an object relative to a robot and an inertial map
% coordinate frame
pr_m0 = [r 0 0]';  % Initial position of robot in map frame
robot_euler0 = [0 pi/4 0]; % Initial orientation of robot relative to map x axis
po_m = [0 0 r]';  % Position of object in robot frame (height z = 0)

axis_l = 10; % Length of axes to plot
%Set up Inertial axes
XYZ = eye(3);
% Set up aerospace axes
NED = axis_l*[ 1 0 0; 0 -1 0; 0 0 -1];

for t=0:T
    
    % Construct circular inward looking trajectory
    pr_m = pr_m0 + r*[cos(2*pi/T*t)-1; sin(2*pi/T*t); 0]; 
    robot_euler = robot_euler0 + [0 0 -2*pi/T*t];

    % Construct 3D rotation matrix to convert from robot to map frame using
    % aerospace convention
    Rr2m = (rot(robot_euler(1),1)'*rot(robot_euler(2),2)'*rot(robot_euler(3),3)')'; % 3D rotation matrix
    Rr2mSC = SpinCalc('EA123toDCM', radtodeg([robot_euler(1), robot_euler(2), robot_euler(3)]));    
    Rr2m-Rr2mSC
    
    % Find position of object in map frame using equation on slide 8 of transforms
    % p^o_m = (R^m_r)*p^o_r + p^r_m
    po_r = Rr2m'*(po_m - pr_m);

    figure(1);clf; hold on;
    % Draw Map axes
    plot3( [0 NED(1,1)], [ 0 NED(1,2)], [ 0 NED(1,3)], 'b') ; % Map North axis
    xlabel('Map North axis')
    plot3( [0 NED(2,1)], [0 NED(2,2)], [0 NED(2,3)], 'b'); % Map East axis
    ylabel('Map East axis')
    plot3( [0 NED(3,1)], [0 NED(3,2)], [0 NED(3,3)], 'b'); % Map Down axis
    zlabel('Map Down axis')
    text( 1,0.5, 0, 'Map axes');
    grid on;
    axis equal;
    view(0, 0);

    % Draw Robot, Object in Map
    load('quad_starmac.mat')
    drawquadrotor(qm_fnum, qm_xyz, pr_m(1), pr_m(2), pr_m(3), robot_euler(1), robot_euler(2), robot_euler(3),[0.6 0.1 0.1]);
    plot3(po_m(1), po_m(2), po_m(3), 'go', 'MarkerSize', 6, 'LineWidth', 2); % Draw object

    % Object measurement line from robot in map frame
    plot3([pr_m(1) po_m(1)], [pr_m(2) po_m(2)], [pr_m(3) po_m(3)], 'b'); 

    % Draw Robot axes
    robotaxisep = Rr2m*NED + [pr_m pr_m pr_m];% Robot axis end points
    plot3( [pr_m(1) robotaxisep(1,1)], [pr_m(2) robotaxisep(2,1)],[pr_m(3) robotaxisep(3,1)], 'r') ;
    plot3( [pr_m(1) robotaxisep(1,2)], [pr_m(2) robotaxisep(2,2)],[pr_m(3) robotaxisep(3,2)], 'r') ;
    plot3( [pr_m(1) robotaxisep(1,3)], [pr_m(2) robotaxisep(2,3)],[pr_m(3) robotaxisep(3,3)], 'r') ;
    text( pr_m(1)+1,pr_m(2)+1, pr_m(3)+1, 'Robot axes');

    pause(0.01);

end

