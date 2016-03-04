% Each "example" is basically just a simulation loop that moves the robot 
% and plots the results. The different examples are just different
% combinations of sensor arrangements and environments.
% The example loads a map and robot starting position, then defines the
% sensors used, and then runs a simulation loop. The mapping algorithm 
% function is called from within the simulation loop.

% Occupancy Grid Mapping
clear; clc;

%% Create AVI object
makemovie1 = 1; % Inverse Measurement Model Video
if(makemovie1)
    vidObj1 = VideoWriter('inversemeasbres.avi');
    vidObj1.Quality = 100;
    vidObj1.FrameRate = 4;
    open(vidObj1);
end
makemovie2 = 1; % Inverse Measurement Model Video
if(makemovie2)
    vidObj2 = VideoWriter('mapbres.avi');
    vidObj2.Quality = 100;
    vidObj2.FrameRate = 4;
    open(vidObj2);
end

% Simulation time
Tmax = 150;
T = 0:Tmax;

% Load map
[map, M, N, x0] = loadmap(2);

% Robot motions
u = [3 0 -3 0;
     0 3 0 -3];
ui=1;

% Robot sensor rotation command
w = 0.3*ones(length(T));

% Belief map
m = 0.5*ones(M,N);
L0 = log(m./(1-m));
L=L0;

% Sensor model parameters
meas_phi = [-.4:0.01:.4]; % Measurement bearings
rmax = 30; % Max range
% --- alpha and beta never used????
alpha = 1; % Width of an obstacle (Distance about measurement to fill in)
beta = 0.05; % Width of a beam (Angle beyond which to exclude) 

%State Initialization
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
    [L, measL, meas_r] = ogmap(map, L, x(:, t), meas_phi, rmax, 0);

    % Calculate probabilities
    m = exp(L)./(1+exp(L));
    invmod_T = exp(measL)./(1+exp(measL));

    %% Plot results
    
    % Map and vehicle path
    figure(1);clf; hold on;
    image(100*(1-map));
    colormap(gray);
    plot(x(2,1:t),x(1,1:t),'bx-')
    axis([0 N 0 M])

    % Inverse measurement model
    figure(2);clf; hold on;
    image(100*(invmod_T));
    colormap(gray);
    plot(x(2,t),x(1,t),'bx')
    for i=1:length(meas_r)
        plot( x(2,t)+meas_r(i)*sin(meas_phi(i) + x(3,t)),x(1,t)+meas_r(i)*cos(meas_phi(i)+ x(3,t)),'ko')
    end
    axis([0 N 0 M])
    %F2(t-1) = getframe;
    title('Measurements and inverse measurement model');
    if (makemovie1) writeVideo(vidObj1, getframe(gca)); end

    % Belief map
    figure(3);clf; hold on;
    image(100*(m));
    colormap(gray);
    plot(x(2,max(1,t-10):t),x(1,max(1,t-10):t),'bx-')
    axis([0 N 0 M])
    %F3(t-1) = getframe;
    title('Current occupancy grid map')
    if (makemovie2) writeVideo(vidObj2, getframe(gca)); end

end
if (makemovie1) close(vidObj1); end
if (makemovie2) close(vidObj2); end
