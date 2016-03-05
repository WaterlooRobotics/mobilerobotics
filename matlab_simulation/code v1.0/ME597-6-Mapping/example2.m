% Each example is basically just a simulation loop that moves the robot 
% and plots the results. The different examples are just different
% combinations of sensor arrangements and environments.
% The example loads a map and robot starting position, then defines the
% sensors used, and then runs a simulation loop. The mapping algorithm 
% function is called from within the simulation loop.

%% Occupancy grid mapping example 1
% Uses a 100 x 100 cell map with a laser scanner, map is updated using
% Bresenham ray trace mode.

clear; clc;

% Add libraries
addpath('../../04-sensor');

%% Create AVI object
makemovie1 = 1; % Inverse Measurement Model Video
if(makemovie1)
    vidObj1 = VideoWriter('example2_imm.avi');
    vidObj1.Quality = 100;
    vidObj1.FrameRate = 4;
    open(vidObj1);
end
makemovie2 = 1; % Inverse Measurement Model Video
if(makemovie2)
    vidObj2 = VideoWriter('example2_map.avi');
    vidObj2.Quality = 100;
    vidObj2.FrameRate = 4;
    open(vidObj2);
end

% Simulation time
Tmax = 150;
T = 0:Tmax;

% Load map
[map, M, N, x0] = load_cell_map(1);

% Robot motions
u = [3 0 -3 0;
     0 3 0 -3];
ui=1;

% Robot sensor rotation command
w = 0.3*ones(length(T));

% Occupancy grid in both probablity and log odds form
og = 0.5*ones(M,N);
ogl0 = log(og./(1-og));
ogl=ogl0;

% Sensor model parameters - this example uses a sonar sensor
phi_m = 0; % Measurement bearings
r_max = 20; % Max range
alpha = 1; % Width of an obstacle (Distance about measurement to fill in)
beta = 1.5; % Width of a beam (Angle beyond which to exclude) 

% State Initialization
x = zeros(3,length(T)+1);
x(:,1) = x0;

%% Main simulation
for t=2:length(T)
    % Robot motion
    move = x(1:2,t-1) + u(:,ui)
	% If the robot hits a wall or obstacle, change direction
    if ((move(1)>M||move(2)>N||move(1)<1||move(2)<1) || (map(move(1),move(2))==1))
        x(:,t) = x(:,t-1);
        ui = mod(ui,4)+1;
    else
        x(1:2,t) = move;
    end
    x(3,t) = x(3,t-1) + w(t);

    %% Map update;
	% Call occupancy grid mapping function
    [ogl, imml, r_m] = ogmap(map, ogl, x(:, t), phi_m, r_max, alpha, beta, 1);

    % Calculate probabilities
    og = exp(ogl)./(1+exp(ogl));
    og_mm = exp(imml)./(1+exp(imml));

    %% Plot results
    
    % Map and vehicle path
    plot_cell_map(map, M, N, 1);
    plot_robot_path(x, t, 1);

    % Inverse measurement model
    plot_inverse_mm(og_mm, M, N, x(:, t), r_m, phi_m, 2);
    if (makemovie1) writeVideo(vidObj1, getframe(gca)); end

    % Belief map
    plot_occupancy_grid(og, M, N, 3);
    if (makemovie2) writeVideo(vidObj2, getframe(gca)); end
    
end
if (makemovie1) close(vidObj1); end
if (makemovie2) close(vidObj2); end
