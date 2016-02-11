clear;clc; close all;
% Different car models, driving around a circle. Types include:
% Dubins - This model uses speed and angular velocity as inputs
% TwoWheel - This model uses left and right wheel speeds as inputs
% Bicycle - This model uses speed and steering angle as inputs, can also
% be used for four wheel Ackermann steered vehicles (like cars)

modeltype = 'Bicycle';
video = false;

%% Create AVI movie object
if video
    vidObj = VideoWriter('car.avi'); % Define filename
    vidObj.Quality = 100; % Set quality
    vidObj.FrameRate = 20; % Set framerate
    open(vidObj); % Open video object
end

%% Simulations
% Time
T = 3; % Duration
dt = 0.01;  % Timestep
tvec = 0:dt:T; % Time vector
n = length(tvec); % Number of timesteps

% Vehicle parameters
r = 1; % Wheel radius
l = 0.2; % Distance from wheel to center

%% Dubins Model Commands
% Speed command
v = .6*ones(1,n); % Constant throughout

% Angular velocity
omega = 2*ones(1,n);
omega(round(end/3):end) = -2;

%% Two Wheel Model Commands
% Left Wheel angular velocity command
phidl = .2*ones(1,n); % Constant throughout
phidl(round(end/3):end) = 1; % Change value part way through

% Wheel 2 angular velocity command
phidr = 1*ones(1,n);
phidr(round(end/3):end) = 0.2;

%% Bicycle Model Commands

% Speed command
v = .6*ones(1,n); % Constant throughout

% Steering Angle
delta = 0.6*ones(1,n);
delta(round(end/3):end) = -0.6;


%% Body motion integration
x = [0; 0; 0];
switch lower(modeltype)
    case 'dubins'
        disp('Dubins Car Simulation')
        for t=1:n
            x(:,t+1) = dubins(x(:,t),v(t),omega(t),dt);
        end
        
    case 'twowheel'
        disp('Two Wheel Car Simulation')
        for t=1:n
            x(:,t+1) = twowheel(x(:,t),phidl(t),phidr(t),r,l,dt);
        end
        
    case 'bicycle'
        disp('Bicycle Car Simulation')
        for t=1:n
            x(:,t+1) = bicycle(x(:,t),v(t),delta(t),l,dt);
        end
end

% Create figure of motion
figure(1);
for t=1:2:n % For every second position of the robot
    clf;hold on; % clear the current figure
    switch lower(modeltype)
        case 'bicycle'
            drawcar(x(1,t),x(2,t),x(3,t),delta(t),0.1,1) % Draw the car
        otherwise
            drawbot(x(1,t),x(2,t),x(3,t),0.1,1) % Draw the robot
    end
    plot(x(1,1:t), x(2,1:t), 'bx'); % Draw the path of the vehicle
    axis equal; % Use same scale on both axes
    axis([-0.2 1 -0.2 1.0]); % Set the dimensions of the plot
    drawnow; % Draw the plot now instead of when Matlab feels like it
    if video 
        writeVideo(vidObj, getframe(gcf)); % Record the plot as a frame in the movie
    end
end

if video
    close(vidObj); % Finish making the movie file
end
