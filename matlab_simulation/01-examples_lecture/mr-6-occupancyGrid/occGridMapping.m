% Each example is basically just a simulation loop that moves the robot 
% and plots the results. The different examples are just different
% combinations of sensor arrangements and environments.
% The example loads a map and robot starting position, then defines the
% sensors used, and then runs a simulation loop. The mapping algorithm 
% function is called from within the simulation loop.

%% Occupancy grid mapping example 1
% Uses a 50 x 60 cell map with a laser scanner, map is updated using
% Bresenham ray trace mode.

clear; clc;

%% Create AVI object
makemovie1 = 0; % Inverse Measurement Model Video
if(makemovie1)
    vidObj1 = VideoWriter('ex1_measurement_model.avi');
    vidObj1.Quality = 100;
    vidObj1.FrameRate = 4;
    open(vidObj1);
end
makemovie2 = 0; % Occupancy Grid Video
if (makemovie2)
    vidObj2 = VideoWriter('ex1_occupancy_grid.avi');
    vidObj2.Quality = 100;
    vidObj2.FrameRate = 4;
    open(vidObj2);
end

%% Select simulation example
% 0 - Default example, 50X60 map, Bresenham's update with Lidar
% 1 - Default example, 50X60 map, windowed update with Lidar
% 2 - Default example, 50X60 map, windowed update wth sonar
% 3 - Large scale 1000X1000 map, Bresenham's update with Lidar

example = 0; 

% Sensor model parameters - this example uses a lidar
if (example ~= 2)
    % Lidar model
    phi_m = -.4:0.01:.4; % Measurement bearings
    r_max = 30; % Max range
    alpha = 1; % Width of an obstacle (Distance about measurement to fill in)
    beta = 0.05; % Width of a beam (Angle beyond which to exclude)
    % Probabilites of cells
    p_occ = 0.8;
    p_free = 0.3;
else
    % Sonar model
    phi_m = 0; % Measurement bearing
    fov = 60*pi/180; %  Field of view of sonar in rad 
    r_max = 30; % Max range
    alpha = 1; % Width of an obstacle (Distance about measurement to fill in)
    beta = fov; % Width of a beam in rad (Angle beyond which to exclude)
    % Probabilites of cells
    p_occ = 0.6;
    p_free = 0.2;
end

% Simulation time
Tmax = 250;
T = 0:Tmax;

% State Initialization (x, y, theta, psi)
% x,y position
% theta robot heading (absolute)
% psi sensor heading (absolute, but could be relative to robot)
x0 = [25; 10; 0; 0];
x = zeros(4, length(T) + 1);
x(:, 1) = x0;

% Robot velocity command
v = 3;

% Robot sensor rotation command
w = 0.6;

% Load map - binary, 1 = occupied
[map, M, N] = load_cell_map(1);

% Occupancy grid in both probablity and log odds form
og = 0.5*ones(M, N);
ogl0 = log(og./(1-og));
ogl = ogl0;


%% Main simulation
for t = 2:length(T)+1
    %% Simulation
    % Robot motion
    [x(:,t)] = udlr_motion(map, x(:,t-1), v);
    x(4,t) = x(4,t-1) + w;

    % Generate a measurement data set
    if (example == 2)
        r_m = get_sonar_range(map, x([1:2,4],t), phi_m, fov, r_max)
    else
        r_m = getranges(map, x([1:2,4],t), phi_m, r_max);
    end
    
    %% Map update;
	% Call occupancy grid mapping logit update function using Bresenham
    [ogl, imml] = ogmap_update(ogl, x(:,t), phi_m, r_m, r_max, alpha, beta, p_occ, p_free, example);

    % Calculate probability map from log odds map
    og = exp(ogl)./(1 + exp(ogl));
    og_mm = exp(imml)./(1 + exp(imml));

    %% Plot results
    
    % Map and vehicle path
    plot_cell_map(map, M, N, 1);
    plot_robot_path(x, t, 1);

    % Inverse measurement model
    plot_inverse_mm(og_mm, M, N, x(:, t), phi_m, r_m, 2);
    if (makemovie1) writeVideo(vidObj1, getframe(gca)); end

    % Belief map
    plot_occupancy_grid(og, M, N, 3);
    plot_robot_path(x, t, 3);
    if (makemovie2) writeVideo(vidObj2, getframe(gca)); end
   
    drawnow;
end

if (makemovie1) close(vidObj1); end
if (makemovie2) close(vidObj2); end
