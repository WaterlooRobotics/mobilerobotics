% Nonlinear programming examples
clear all; close all; clc

global n m N T dt xd obs pF withobs endonly

%% Optimization problem definition
% Number of optimization variables per timestep: n states and m inputs
n = 3;
m = 2;
N = (n+m);

% Example #1 Basic tracking of sinusoidal desired trajectory
% Time steps
T = 10;
dt = 1;
% Desired trajectory
xd = [0:dt:(T-1)*dt; sin(0.3*[0:dt:(T-1)*dt]);zeros(size(0:dt:(T-1)*dt))]';
%Initial position
p0 = [0 2 0];

% Example #2 Tracking of sinusoidal desired trajectory with obstacles
% Time steps
withobs = 1;
if (withobs)
    T = 40;
    dt = 0.2;
    % Desired trajectory
    xd = [0:dt:(T-1)*dt; sin(0.3*[0:dt:(T-1)*dt]);zeros(size(0:dt:(T-1)*dt))]';
    %Initial position
    p0 = [0 2 0];
    % Set up environment
    posMinBound = [0 -1];
    posMaxBound = [4 3];
    numObsts = 6;
end

% Example #3 Navigating to a destination (with or without obstacles from
% above)
endonly = 0;
if (endonly)
    T = 10;
    dt = 1;
    pF = [ 4 0 0.5];
end 

% Initial solution
x0 = zeros(N*T,1);
x0(1:N:end) = p0(1);
x0(2:N:end) = p0(2);
x0(3:N:end) = p0(3);
x0(4:N:end) = 0;
x0(5:N:end) = 0;

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


figure(1); clf; hold on;
if (~endonly)
    plot(xd(:,1), xd(:,2), 'ro--')
else
    plot(pF(1),pF(2),'ro')
end
if (withobs)
        for i=1:numObsts
        plot(obs(i,1), obs(i,2),'bx');
        circle(1, obs(i,:), radius(i));
    end
    axis equal
end
drawnow();

% Solve nonlinear program
options = optimset('display', 'off','maxfunevals',50000);
tic;
[X,FVAL,EXITFLAG,OUTPUT,LAMBDA] = FMINCON(@(x) cost(x),x0,A,B,Aeq,Beq,LB,UB,@(x) constraints(x), options);
toc;
% Rename results
x = X(1:N:end);
y = X(2:N:end);
th = X(3:N:end);
v = X(4:N:end);
w = X(5:N:end);

% Plot results
figure(2);clf; hold all;
plot(1:T,x)
plot(1:T,y)
plot(1:T,th)
plot(1:T,v)
plot(1:T,w)

figure(1); hold on;
plot(x,y,'bx-');
