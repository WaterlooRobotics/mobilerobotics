% Deterministic and Random Sampling for maze solving
close all; clc; clear;

% Compute time check (Waslander Lenovo W530 = 0.39-0.46 s)
tic;
disp('Speed test: 0.39s on Lenovo W530.  This machine:');
inv(rand(2000));
toc;

% Each row of the map structure draws a single line of the maze
% with coordinates [x1 x2 y1 y2];
% Top left corner of maze is [0.5 0.5], 
% Bottom right corner is [col+0.5
% row+0.5]
row = 15;
col = 15;
xMin = 0.5;
yMin = 0.5;
xMax = col+0.5;
yMax = row+0.5;
map = maze(row,col,'C');
start = [0.5, 1.0];
finish = [col+0.5, row];

% Sample points
figure(1); hold on;
plot(start(1), start(2), 'gx')
plot(finish(1), finish(2), 'rx')

% Create Milestones
tic;
nS = 2500;
milestones = [row*rand(nS,1)+xMin col*rand(nS,1)+xMin];
milestones(nS+1,:) = start;
milestones(nS+2,:) = finish;
nM = nS+2;
disp('Time to create milestones');
toc;
plot(milestones(:,1),milestones(:,2),'m.');
drawnow;

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
            if (~CheckCollisionMaze(milestones(i,:),milestones(cur,:), map))
                e(i,cur) = 1;
                e(cur,i) = 1;
                plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
            end
        end
    end
end
disp('Time to find edges');
toc;
drawnow;

% Find shortest path
tic;
[sp, sd] = shortestpath_mr(milestones, e, nM-1, nM, 1, 1, 0);
if length(sp) == 0
    disp('Failed to find path')
else
    for i=1:length(sp)-1
        plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
    end
    title('Solved Maze');
    disp('Time to find shortest path');
end
toc;

