% Robot trajectories
% This code only focused on generating different trajectories. 
% Path planning is not considered.
% This code can plot six different trajectories:
% spiral, nudges, swerves, corner, lane change, obstacle avoid.
% In addition, it can plot the trajectory which is defined by users.
% Users are also allowed to give any initial condition except number 6.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
disp('5: A single lane change')
disp('6: Get around an obstacle')
disp('7: User defined motion model')
prompt = '';
index = input(prompt);

% User input check
% Input index should be between 1-5
index = input_check(index);

if index == 6
    % In this program we don't focus on the path planning.
    % The initial condition for 6 is fixed.
    x0 = [1; 1; 1];
else
    % Let user define an initial condition
    disp('Please give an initial condition, i.e. [1 1 1].')
    disp('x1: x-position; x2: y-position; x3: heading angle.')
    prompt = '';
    x0 = input(prompt)';
end

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
        for i = 1: 5
            [xddot, xd]=nudges(T, dt, xd, xddot, i);
            plot_figure(xd, T, index);
        end
    case 3
        % Swerves
        for i = 1: 5
            [xddot, xd]=swerves(T, dt, xd, xddot, i);
            plot_figure(xd, T, index);
        end
    case 4
        % Corner
        [xddot, xd]=corner(T, dt, xd, xddot);
        plot_figure(xd, T, index);
    case 5
        % A single lane change
        [xddot, xd]=lane_change(T, dt, xd, xddot);
        plot_figure(xd, T, index);
    case 6
        % Get around an obstacle
        [xddot, xd]=obstacle(T, dt, xd, xddot);
        plot_figure(xd, T, index);
    case 7
        % User defined motion model
        [xddot, xd] = user_define_motion_model(T, dt, xd, xddot);
        plot_figure(xd, T, index);
end
