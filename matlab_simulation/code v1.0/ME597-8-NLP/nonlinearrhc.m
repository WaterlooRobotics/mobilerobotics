% Nonlinear programming receding horizon examples
clear all; close all; clc

global n m N T dt xd obs pF withobs endonly

%% Optimization problem definition
% Number of optimization variables per timestep: n states and m inputs
n = 3;
m = 2;
N = (n+m);

% Example #1 Tracking a trajectory 
% Time steps
TTot = 20;
dt = .3;
% Desired trajectory
xdT = [0:dt:(TTot-1)*dt; sin(0.3*[0:dt:(TTot-1)*dt]);zeros(size(0:dt:(TTot-1)*dt))]';
%Initial position
p0 = [0 2 0];
% Set up environment
posMinBound = [0 -1];
posMaxBound = [6 3];
numObsts = 6;
withobs = 1;

% Example #2 Navigating to a destination 
endonly = 0;
if (endonly)
    TTot = 10;
    dt = 1;
    pF = [ 4 0 0.5];
end 

% Receding Horizon
T = 5;

% Constraints
A = [];
B = [];
Aeq = zeros(3,N*T);
Aeq(1:3,1:3) = eye(3);
Beq = p0';

% State and input bounds
LB = -100*ones(N*T,1);
LB(4:N:end) = 0;
LB(5:N:end) = -1.5;

UB = 100*ones(N*T,1);
UB(4:N:end) = 2;
UB(5:N:end) = 1.5;


% Define random round obstacles
if (withobs)
    range = (posMaxBound-posMinBound);
    obs = rand(numObsts,2);
    obs(:,1) = posMinBound(1)+range(1)*obs(:,1);
    obs(:,2) = posMinBound(2)+range(2)*obs(:,2);
    for i=1:numObsts
        for j=1:numObsts
            dist(i,j) = norm(obs(i,:)-obs(j,:));
        end
        dist(i,numObsts+1) = norm(obs(i,:)-p0(1:2));
        radius(i) = min(dist(i,[1:i-1 i+1:end]))/2.5;
    end
    obs = [obs radius'];
end
% Initial solution
x0 = zeros(N*T,1);
x0(1:N:end) = p0(1);
x0(2:N:end) = p0(2);
x0(3:N:end) = p0(3);
x0(4:N:end) = 0;
x0(5:N:end) = 0;

% Receding horizon goal
xd = xdT(1:T,:)

% Repeat optimization at each timestep
for i=1:TTot-T

    % Solve nonlinear program
    options = optimset('maxfunevals',50000);
    tic;
    [X,FVAL,EXITFLAG,OUTPUT,LAMBDA] = fmincon(@(x) cost(x),x0,A,B,Aeq,Beq,LB,UB,@(x) constraints(x), options);
    toc;
    % Move and Update problem for next timestep
    x0(1:N*(T-1)) = X(N+1:N*T,1);
    x0(N*(T-1)+1:N*T) = X(N*(T-1)+1:N*T); % should be propagating dynamics here.
    xd = xdT(i+1:i+T,:);
    Beq = X(N+1:N+n)';

    % Rename results
    x = X(1:N:end);
    y = X(2:N:end);
    th = X(3:N:end);
    v = X(4:N:end);
    w = X(5:N:end);

    % Plot results
%     figure(2);clf; hold all;
%     plot(1:T,x)
%     plot(1:T,y)
%     plot(1:T,th)
%     plot(1:T,v)
%     plot(1:T,w)

    figure(i); clf; hold on;
    plot(x,y,'bx-');
    if (~endonly)
        plot(xdT(1:end-1,1), xdT(1:end-1,2), 'ro--')
    else
        plot(pF(1),pF(2),'ro')
    end
    if (withobs)
        for j=1:numObsts
            plot(obs(j,1), obs(j,2),'bx');
            circle(i, obs(j,:), radius(j));
        end
        axis equal
    end
    drawnow();
    F(i) = getframe;
end
