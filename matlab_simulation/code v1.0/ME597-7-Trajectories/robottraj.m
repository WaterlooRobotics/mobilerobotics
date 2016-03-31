% Robot trajectories
clear;
close;
clc;

% Time
Tmax = 10;
dt = 0.1;
T = 0:dt:Tmax;

% Let user choose the input trajectory
disp('What is the input trajectory?')
disp('1: Spiral; 2: Nudges; 3: Swerves; 4: Corner;')
disp('5: User defined motion model')
prompt = '';
index = input(prompt);

% User input check
% Input index should be between 1-5
index = input_check(index);

% Let user define an initial condition
disp('Please give an initial condition, i.e. [1 1 1].')
disp('x1: x-position; x2: y-position; x3: heading angle.')
prompt = '';
x0 = input(prompt)';

% Define the states
% xd1: x-position; xd2: y-position; xd3: heading angle
% xddot1: x-velocity; xddot2: y-velocity; xddot3: angular velocity
xd = zeros(3,length(T)+1);
xddot = zeros(3,length(T));

% Set the initial condition as what user defined
xd(:,1)= x0;

switch index
    case 1
        % Spiral
        [xddot, xd]=spiral(T, dt, xd, xddot);
        plot_figure(xd, T, index);
        
    case 2
        % Nudges
        [xddot, xd]=nudges(T, dt, xd, xddot);
        plot_figure(xd, T, index);
    case 3
        % Swerves
        [xddot, xd]=swerves(T, dt, xd, xddot);
        plot_figure(xd, T, index);
    case 4
        % Corner
        [xddot, xd]=corner(T, dt, xd, xddot);
        plot_figure(xd, T, index);
    case 5
        % User defined motion model
        [xddot, xd] = user_define_motion_model(T, dt, xd, xddot);
        plot_figure(xd, T, index);
end