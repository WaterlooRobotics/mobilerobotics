%% 3D transformation demo
% Demonstrates conversions between 123-Euler angles, and direction cosine matrices, and
% by moving a quadrotor through a circular trajectory.

clc; clear; 

T = 100; % Timesteps
r = 2; % Radius of circle flown

% Define the position of an object relative to a robot and an inertial map
% coordinate frame
pr_m0 = [r 0 0]';  % Initial position of robot in map frame
robot_euler0 = [pi/4 pi/4 pi/4]; % Initial orientation of robot relative to map x axis
po_r = [0 0 r]';  % Position of object in robot frame (height z = 0)

%Set up Inertial axes
XYZ = eye(3);
axis_l = 5; % Length of axes to plot
MAP = axis_l*XYZ;

for t=0:T
    
    % Construct circular inward looking trajectory
    % Rotate robot position in map frame about z axis 
    pr_m = rot(-2*pi*t/T,3)*pr_m0; % Take nominal position in robot frame, rotate to map frame, hence -sign
    pr_mS(:,t+1) = pr_m; % store
    
    % Change robot heading to always point outward
    robot_euler = robot_euler0 + [0 0 2*pi/T*t]

    % Construct 3D rotation matrix to convert from map to robot frame using
    % aerospace convention
    Rm2r = (rot(robot_euler(1),1)*rot(robot_euler(2),2)*rot(robot_euler(3),3)); % 3D rotation matrix
    % Find position of object in map frame using equation on slide 8 of transforms
    % p^o_m = (R^m_r)*p^o_r + p^r_m
    po_m = Rm2r'*po_r + pr_m;
    po_mS(:,t+1) = po_m; % store

    figure(1); clf; hold on;
    % Draw Map axes
    plot3( [0 MAP(1,1)], [ 0 MAP(1,2)], [ 0 MAP(1,3)], 'b') ; % Map X axis
    plot3( [0 MAP(2,1)], [0 MAP(2,2)], [0 MAP(2,3)], 'r'); % Map Y axis
    plot3( [0 MAP(3,1)], [0 MAP(3,2)], [0 MAP(3,3)], 'g'); % Map Z axis
    xlabel('Map X axis')
    ylabel('Map Y axis')
    zlabel('Map Z axis')
    %text( 1,0.5, 0, 'Map axes');
    
    % Draw Robot axes
    robotaxisep = Rm2r'*MAP + [pr_m pr_m pr_m];% Robot axis end points
    plot3( [pr_m(1) robotaxisep(1,1)], [pr_m(2) robotaxisep(2,1)],[pr_m(3) robotaxisep(3,1)], 'b') ;
    plot3( [pr_m(1) robotaxisep(1,2)], [pr_m(2) robotaxisep(2,2)],[pr_m(3) robotaxisep(3,2)], 'r') ;
    plot3( [pr_m(1) robotaxisep(1,3)], [pr_m(2) robotaxisep(2,3)],[pr_m(3) robotaxisep(3,3)], 'g') ;
    %text( pr_m(1)+1,pr_m(2)+1, pr_m(3)+1, 'Robot axes');

    % Draw Robot path
    plot3( pr_mS(1,:),pr_mS(2,:), pr_mS(3,:), 'Color',[0.6 0.1 0.1]) ;
    
    % Draw Robot, Object in Map
    load('quad_starmac.mat')
    drawquadrotor(qm_fnum, qm_xyz, pr_m(1), pr_m(2), pr_m(3), robot_euler(1), robot_euler(2), robot_euler(3),[0.6 0.1 0.1]);
    plot3(po_m(1), po_m(2), po_m(3), 'co', 'MarkerSize', 6, 'LineWidth', 2); % Draw object

    % Object measurement line from robot in map frame
    plot3([pr_m(1) po_m(1)], [pr_m(2) po_m(2)], [pr_m(3) po_m(3)], 'c'); 
    % Draw object path
    plot3( po_mS(1,:),po_mS(2,:), po_mS(3,:), 'c') ;

    grid on;
    axis equal;
    view (15,15)
    pause(0.01);

end


% Repeat in aerospace axes 
NED_conv = [ 0 1 0; 1 0 0; 0 0 -1]
NED = axis_l*NED_conv;
% Convert coordinates to NED
pr_m0_NED = NED_conv*pr_m0;  % Initial position of robot in map frame
robot_euler0_NED = NED_conv*robot_euler0'; % Initial orientation of robot relative to map x axis
po_r = NED_conv*po_r;  % Position of object in robot frame (height z = 0)

%Clear storage
pr_mS = []; po_mS = [];

for t=0:T
    
    % Construct circular inward looking trajectory
    % Rotate robot position in map frame about z axis 
    pr_m = rot(-2*pi*t/T,3)*pr_m0; % Take nominal position in robot frame, rotate to map frame, hence -sign
    pr_mS(:,t+1) = pr_m; % store
    
    % Change robot heading to always point outward
    robot_euler = robot_euler0 + [0 0 2*pi/T*t]

    % Construct 3D rotation matrix to convert from map to robot frame using
    % aerospace convention
    Rm2r = (rot(robot_euler(1),1)*rot(robot_euler(2),2)*rot(robot_euler(3),3)); % 3D rotation matrix
    % Find position of object in map frame using equation on slide 8 of transforms
    % p^o_m = (R^m_r)*p^o_r + p^r_m
    po_m = Rm2r'*po_r + pr_m;
    po_mS(:,t+1) = po_m; % store

    figure(1);clf; hold on;
    % Draw Map axes
    plot3( [0 NED(1,1)], [0 NED(1,2)], [0 NED(1,3)], 'b') ; % Map X axis
    plot3( [0 NED(2,1)], [0 NED(2,2)], [0 NED(2,3)], 'r'); % Map Y axis
    plot3( [0 NED(3,1)], [0 NED(3,2)], [0 NED(3,3)], 'g'); % Map Z axis
    xlabel('Map X axis')
    ylabel('Map Y axis')
    zlabel('Map Z axis')
    %text( 1,0.5, 0, 'Map axes');
    
    % Draw Robot axes
    robotaxisep = Rm2r'*NED + [pr_m pr_m pr_m];% Robot axis end points
    plot3( [pr_m(1) robotaxisep(1,1)], [pr_m(2) robotaxisep(2,1)],[pr_m(3) robotaxisep(3,1)], 'b') ;
    plot3( [pr_m(1) robotaxisep(1,2)], [pr_m(2) robotaxisep(2,2)],[pr_m(3) robotaxisep(3,2)], 'r') ;
    plot3( [pr_m(1) robotaxisep(1,3)], [pr_m(2) robotaxisep(2,3)],[pr_m(3) robotaxisep(3,3)], 'g') ;
    %text( pr_m(1)+1,pr_m(2)+1, pr_m(3)+1, 'Robot axes');

    % Draw Robot path
    plot3( pr_mS(1,:),pr_mS(2,:), pr_mS(3,:), 'm') ;
    
    % Draw Robot, Object in Map
    load('quad_starmac.mat')
    drawquadrotor(qm_fnum, qm_xyz, pr_m(1), pr_m(2), pr_m(3), robot_euler(1), robot_euler(2), robot_euler(3),[0.6 0.1 0.1]);
    plot3(po_m(1), po_m(2), po_m(3), 'co', 'MarkerSize', 6, 'LineWidth', 2); % Draw object

    % Object measurement line from robot in map frame
    plot3([pr_m(1) po_m(1)], [pr_m(2) po_m(2)], [pr_m(3) po_m(3)], 'c'); 
    % Draw object path
    plot3( po_mS(1,:),po_mS(2,:), po_mS(3,:), 'c') ;
 
    grid on;
    axis equal;
    view (15,15)
    pause(0.01);

end