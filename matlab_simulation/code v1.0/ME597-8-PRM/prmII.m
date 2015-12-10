%% Probabilistic Road Map example
clear; clc;


%% Problem parameters
tic;

% Set up the map
xMax = [32 30]; % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = [25.5 28.5];
xF = [6 1];

% Set up the obstacles
rand('state', 1);
nO = 30; % number of obstacles
nE = 4; % number of edges per obstacle (not changeable).
minLen.a = 1; % Obstacle size bounds
maxLen.a = 4;
minLen.b = 1;
maxLen.b = 6;

obstBuffer = 1; % Buffer space around obstacles
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
disp('Time to create environment');
toc;

%% Multi-query PRM, created until solution found
tic;
done = 0;
milestones = [x0; xF];
e = zeros(2,2);
% Number of edges to add
p = 8;
t= 0;
tm = 0;
te = 0;
ts = 0;
ec = 0;
while ((~done) && (t < 100))
    t=t+1;
    % Get new milestone
    newstone = 0;
    t0 = cputime;
    while (~newstone)
        sample = [xR(1)*rand(1,1)+xMin(1) xR(2)*rand(1,1)+xMin(2)];
        figure(1); hold on;
        plot(sample(:,1),sample(:,2),'k.');
        keep = inpolygon(sample(1), sample(2), env(:,1),env(:,2));
        if (keep)
            milestones = [milestones; sample];
            newstone = 1;
            figure(1); hold on;
            plot(milestones(:,1),milestones(:,2),'m.');
        end
    end
    t1 = cputime;
    tm = tm+t1-t0;
    % Attempt to add closest p edges
    t2 = cputime;
    cur = length(milestones(:,1));
    for i = 1:cur-1
        d(i) = norm(milestones(cur,:)-milestones(i,:));
    end
    [d2,ind] = sort(d);
    % Check for edge collisions (no need to check if entire edge is
    % contained in obstacles as both endpoints are in free space)
    for j=1:min(p,length(ind))
        ec = ec + 1;
        if (~CheckCollision(milestones(cur,:),milestones(ind(j),:), obsEdges))
            e(ind(j),cur) = 1;
            e(cur,ind(j)) = 1;
            plot([milestones(ind(j),1) milestones(cur,1)],[milestones(ind(j),2) milestones(cur,2)],'m');
        else 
            e(ind(j),cur) = 0;
            e(cur,ind(j)) = 0;
        end
    end
    t3 = cputime;
    te = te + t3 - t2;
    % Check if a path from start to end is found
    t4 = cputime;
    [sp, sd] = shortestpath(milestones, e, 1, 2);
    if (sd>0)
        done = 1;
        for i=1:length(sp)-1
        plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
        end
    end
    t5 = cputime;
    ts = ts + t5 - t4;
end
tm
te
ts
ec
disp('Time to find shortest path');
toc;
