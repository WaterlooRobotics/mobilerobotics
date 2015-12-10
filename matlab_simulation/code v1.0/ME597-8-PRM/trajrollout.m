%% Trajectory rollout example
clear; clc;


%% Problem parameters
tic;

% Set up the map
xMax = [32 30]; % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = [25.5 28.5 -3.0];
xF = [6 1];

% Set up the obstacles
%rand('state', 1);
nO = 25; % number of obstacles
nE = 4; % number of edges per obstacle (not changeable).
minLen.a = 1; % Obstacle size bounds
maxLen.a = 6;
minLen.b = 2;
maxLen.b = 8;

obstBuffer = 0; % Buffer space around obstacles
maxCount = 1000; % Iterations to search for obstacle locations

% Find obstacles that fit
[aObsts,bObsts,obsPtsStore] = polygonal_world(xMin, xMax, minLen, maxLen, nO, x0, xF, obstBuffer, maxCount);

% Define single freespace nonconvex polygon by its vertices, each hole
% separated by NaNs
env = [xMin(1) xMin(2);xMin(1) xMax(2);xMax(1) xMax(2);xMax(1) xMin(2); xMin(1) xMin(2)];
obsEdges = [];
figure(1); hold on;
for i=1:nO
    env = [env; NaN NaN; obsPtsStore(:,2*(i-1)+1:2*i);obsPtsStore(1,2*(i-1)+1:2*i)];
    obsEdges = [obsEdges; obsPtsStore(1:nE,2*(i-1)+1:2*i) obsPtsStore([2:nE 1],2*(i-1)+1:2*i)];
end

% Plot obstacles
figure(1); clf; hold on;
plotEnvironment(obsPtsStore,xMin, xMax, x0, xF);
drawnow();
figure(1); hold on;
disp('Time to create environment');
toc;

%# create AVI object
vidObj = VideoWriter('traj_rollout4.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 5;
open(vidObj);


%% Vehicle
dt = 0.01;
uMin = [2 -2]; % bounds on inputs, [velocity, rotation rate]
uMax = [2 2]; % bounds on inputs, [velocity, rotation rate]
uR = uMax-uMin; % range of inputs
sMin = 20; % steps to move
sMax = 100; % steps to compute for rollout
n_traj = 15; % number of trajectories to roll out

%% Trajectory rollout, created until solution found
tic;
done = 0;
x_cur = x0; %current position
t= 0;
T = 100;
f = 1;


while ((~done) && (t < T))
    score_step = inf;
    for i = 1:n_traj
        % Constant speed, linear distribution of turn rates
        input = [uR(1)/2+uMin(1) uR(2)*(i-1)/(n_traj-1)+uMin(2)];
        steps = sMax; 

        % Propagate Dynamics
        x = x_cur;
        for j=2:steps
            x(j,:) = x(j-1,:)+[input(1)*cos(x(j-1,3))*dt input(1)*sin(x(j-1,3))*dt input(2)*dt]; 
        end
        keep = inpolygon(x(:,1), x(:,2), env(:,1),env(:,2));
        
        if (sum(keep)==steps)
            plot(x(:,1),x(:,2),'g');
            % Score the trajectory
            togo_cur = norm(x(end,1:2)-xF);
            obs_dist = inf;
            for k = 1:nO
                obs_dist = min(obs_dist,Dist2Poly(x(end,1),x(end,2),obsEdges(((k-1)*4)+1:4*k,1),obsEdges(((k-1)*4)+1:4*k,2)));
            end
            obs_dist;
            score_cur = togo_cur - 0.1*obs_dist;
            if (score_cur < score_step)
                score_step = score_cur;
                x_new = x(sMin,:);
                x_plot = x;
            end
        else
            plot(x(:,1),x(:,2),'r');
        end
    end
    % Check if no progress is made
    if (x_new==x_cur)
        x_new(3)=x_new(3)-0.1;
    else
        plot(x_plot(:,1),x_plot(:,2),'b');
        plot(x_plot(end,1),x_plot(end,2),'bo');
    end
    drawnow;
    writeVideo(vidObj, getframe(gcf));
    % Check if a path from start to end is found
    if (norm(x_cur(1:2)-xF)<1)
        done = 1;
    end
    x_cur = x_new;
    t=t+1;
end
close(vidObj);

