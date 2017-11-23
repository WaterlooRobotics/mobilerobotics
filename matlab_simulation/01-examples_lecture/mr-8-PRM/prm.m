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

%% Multi-query PRM, created in batch
tic;

% Get milestones
nS = 500;
samples = [xR(1)*rand(nS,1)+xMin(1) xR(2)*rand(nS,1)+xMin(2)];
keep = inpolygon(samples(:,1),samples(:,2), env(:,1),env(:,2));
milestones = [x0; xF; samples(find(keep==1),:)];
figure(1); hold on;
plot(samples(:,1),samples(:,2),'k.');
plot(milestones(:,1),milestones(:,2),'m.');
nM = length(milestones(:,1));
disp('Time to generate milestones');
toc;

% Attempt to add closest p edges
tic;
p = 20;
e = zeros(nM,nM);
D = zeros*ones(nM,nM);
for i = 1:nM
    % Find closest neighbours
    for j = 1:nM
        d(j) = norm(milestones(i,:)-milestones(j,:));
    end
    [d2,ind] = sort(d);
    % Check for edge collisions (no need to check if entire edge is
    % contained in obstacles as both endpoints are in free space)
    for j=1:p
        cur = ind(j);
        if (i<cur)
            if (~CheckCollision(milestones(i,:),milestones(cur,:), obsEdges))
                e(i,cur) = 1;
                e(cur,i) = 1;
                plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
            end
        end
    end
end
disp('Time to connect roadmap');
toc;

% Find shortest path
tic;
[sp, sd] = shortestpath_mr(milestones, e, 1, 2, 4, 1, 0);
for i=1:length(sp)-1
    plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
end
disp('Time to find shortest path');
toc;
