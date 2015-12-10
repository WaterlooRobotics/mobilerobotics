% ME 597 Tutorial #2 Solution

% Clears all variables in memory; closes all open plot windows; clears
% command window text
clear all; close all; clc

% Defines global variables to be available in the cost and constraint
% functions
global Tr nv nx N n xd beta ds dt

%% Optimization problem definition

% Total number of simulation time steps and dt
TTot = 10;
dt = 10;
% Receding Horizon Length
Tr = 5;

% Define the optimization vector as follows
% x = [x^1_0 u^1_0 ... x^n_0 u^n_0 x^1_Tr u^1_Tr ... x^n_Tr u^n_Tr]
% First time step, vehicle 1 states then inputs, then vehicle 2 to N,
% followed by second time step vehicle 1 to N, up to time Tr.
% We will use this throughout to define our cost and constraints properly

% Sizes
nv = 3; %3; %Number of vehicles
nx = 3; % Number of states (x,y, theta)
nu = 2; % Number of inputs (v, omega)
n = (nx+nu); % Total number of optimization variables per vehicle per time step
N = nv*n; % Total number of optimization variables per time step
NT = (Tr+1)*N; % Total number of optimization variables (Tr horizon steps plus initial conditions)

% Desired trajectories, define (x,y,theta) for each vehicle 
% Note these should be at least Ttot + Tr  steps long so there is always
% a complete desired trajectory to use in the receding horizon simulation
xdT = zeros(nx,15,nv); 
xdT(:,:,1) = [ 0:1:14; zeros(1,15); zeros(1,15)]; % Vehicle 1
xdT(:,:,2) = [3*ones(1,15); 3:-1:(3-14) ; 3*pi/2*ones(1,15)]; % Vehicle 2
xdT(:,:,3) = [-1:1:13; -3:0.5:(-3+7) ; pi/6*ones(1,15)]; % Vehicle 2

%Initial positions of the vehicles, use first point of desired trajectory
p0(:,:) = squeeze(xdT(:,1,:));

% Minimum separation distance
ds = 2;

% Cost function tradeoff parameter
beta = 0.8;

% Noise on motion
R = [ 0.1 0 0; 0 0.1 0; 0 0 0.01];
[RE, Re] = eig(R);


%% Constraints

% Linear Inequality Constraints (none)
A = [];
B = [];
% Linear Equality Constraints (the initial positions of the vehicles)
Aeq = zeros(nx*nv,NT);
for i = 1:nv
    % Row index increments by nx for each new vehicle,
    % Column index increments n to select correct values of optimization
    % vector
    Aeq((i-1)*nx+1:i*nx, (i-1)*n +1:(i-1)*n+nx) = eye(nx); 
    % Rows of Beq match rows of Aeq
    Beq((i-1)*nx+1:i*nx) = p0(:,i);
end

% State and input bounds, (if no bounds are given, still a good idea to
% include them but set them outside the problem scope).
LB = -500*ones(NT,1);  % Lower bounds on all variables
LB(4:n:end,1) = 0.050; % Lower bound on velocity (in km/s)
LB(5:n:end,1) = -0.03; % Lower bound on turn rate (in rad/s)

UB = 500*ones(NT,1);  % Upper bounds on all variables
UB(4:n:end,1) = 0.150; % Upper bound on velocity (in km/s)
UB(5:n:end,1) = 0.03; % Upper bound on turn rate (in rad/s)


% Initial solution, set the entire solution to be at the initial position
% with inputs.  Note this is not actually feasible, but Matlab can handle
% this.  It would be better to come up with feasible solutions, but in the
% collision avoidance problem, this is hard to do.
x0 = zeros(NT,1);
for i = 1:nv
    % The row index picks the appropriate set of n optimization variables
    % for vehicle i, then sets the x,y,theta,v and omega values the same at
    % every time step (incremented by N).
    x0((i-1)*n+1:N:end) = p0(1,i); % x position
    x0((i-1)*n+2:N:end) = p0(2,i); % y position
    x0((i-1)*n+3:N:end) = p0(3,i); % theta angle
    x0((i-1)*n+4:N:end) = 0; % v speed
    x0((i-1)*n+5:N:end) = 0; % omega turn rate
end

% Receding horizon goal, update the subset of the desired trajectory to be
% used for this iteration of the receding horizon optimization
xd(:,:,:) = xdT(:,2:Tr+1,:);

%% Main loop for questions 
% repeats receding horizon optimization at each timestep
for t=2:TTot

    % Solve nonlinear program
    options = optimset('algorithm', 'interior-point','maxfunevals',50000); % Limit how long your optimization can run for
    tic; % used to time the optimization, starts the timer
    % The actual optimization call should not change if everything is
    % defined correctly above
    [X,FVAL,EXITFLAG,OUTPUT,LAMBDA] = fmincon(@(x) cost(x),x0,A,B,Aeq,Beq,LB,UB,@(x) constraints(x), options);
    toc; % returns the elapsed time
    
    % Move one step along the optimal path, and then update problem formulation 
    % to be solved at the next timestep
    % This involves changing 1) initial solution x0, 2) initial condition
    % constraints, 3) desired receding horizon trajectories
    
    % 1) Initial solution update: shift solution one timestep forward, then
    % extend the end of the solution by repeating the last point
    x0(1:N*Tr) = X(N+1:N*(Tr+1)); % Move points 2 to Tr for all vehicles to 1 to Tr-1
    x0(N*Tr+1:N*(Tr+1)) = X(N*Tr+1:N*(Tr+1)); % Duplicate old Tr, should really be propagating dynamics here.

    % 2) Update initial conditions, add in noise for this question
    for i=1:nv
        e = RE*sqrt(Re)*randn(nx,1); % Generate random disturbance based on covariance R
        Beq((i-1)*nx+1:i*nx) = x0((i-1)*n+1:(i-1)*n+nx); % + e; % Add disturbance to new initial positions
    end
    
    % 3) Update desired receding horizon trajectories by moving along full
    % desired trajectory
    xd(:,:,:) = squeeze(xdT(:,t+1:Tr+t,:));

    % Plot results
    figure(1); clf; hold on; % Select figure 1, clf clears the figure, hold on plots multiple things at once without erasing
    color = {'b', 'r','g','m','c'};
    colorx = {'bx-', 'rx-','gx-','mx-','cx-'};
    coloro = {'bo--', 'ro--','go--','mo--','co--'};
    for i=1:nv
        plot(X(n*(i-1)+1:N:end), X(n*(i-1)+2:N:end), colorx{i});
        plot(xdT(1,:,i), xdT(2,:,i),coloro{i})
    end
    axis([-10 10 -10 10]); % Fixes the extent of the scales to [xmin xmax ymin ymax]
    axis equal; % Makes scales equal on both axes
    drawnow;
    pause;
end

