% Deterministic and Random Sampling for maze solving
close all; clc; clear;

% Compute time check (Waslander Lenovo W530 = 0.39-0.46 s)
tic;
disp('Speed test: 0.39 on Lenovo W530.  This machine:');
inv(rand(2000));
toc;

% Each row of the map structure draws a single line of the maze
% with coordinates [x1 x2 y1 y2];
% Top left corner of maze is [0.5 0.5], 
% Bottom right corner is [col+0.5
% row+0.5]
row = 100;
col = 100;
xMin = 0.5;
yMin = 0.5;
xMax = col+0.5;
yMax = row+0.5;
map = maze(row,col, 'C');
start = [0.5, 1.0];
finish = [col+0.5, row];

% Sample points
[rr,cc]=meshgrid(1:col,1:row);
figure(1); hold on;
% plot(rr, cc, 'cx')
plot(start(1), start(2), 'gx')
plot(finish(1), finish(2), 'rx')

% Create Milestones
tic;
nS = row*col;
xlocs = 1:col;
ylocs = 1:row;
milestones(1,:) = start;
milestones(2:nS+1,:) = combvec(xlocs,ylocs)';
milestones(nS+2,:) = finish;
nM = nS+2;
disp('Time to create milestones')
plot(milestones(:,1),milestones(:,2),'m.');
drawnow;

tic;
e = zeros(nM,nM);
D = zeros*ones(nM,nM);
for i = 1:nM
    % closest: [right left up down]

    right = i+1;
    if(mod((i-1),col) ~= 0 && right > 0 && right < nM)
        j = right;
        if (j > 0 && j < nM)
            if (~CheckCollisionSquare2(milestones(i,:),milestones(j,:), map))
                e(i,j) = 1;
                e(j,i) = 1;
                plot([milestones(i,1) milestones(j,1)],[milestones(i,2) milestones(j,2)],'m');
            end
        end
    end
    
    up = i+col;
    if(i ~= 1 && up > 0 && up < nM)
        j = up;
        if (j > 0 && j < nM)
            if (~CheckCollisionSquare2(milestones(i,:),milestones(j,:), map))
                e(i,j) = 1;
                e(j,i) = 1;
                plot([milestones(i,1) milestones(j,1)],[milestones(i,2) milestones(j,2)],'m');
            end
        end
    end
end
i = nM-1;
j = nM;
if (~CheckCollisionSquare2(milestones(i,:),milestones(j,:), map))
    e(i,j) = 1;
    e(j,i) = 1;
    plot([milestones(i,1) milestones(j,1)],[milestones(i,2) milestones(j,2)],'m');
end
i = 1;
j = 2;
if (~CheckCollisionSquare2(milestones(i,:),milestones(j,:), map))
    e(i,j) = 1;
    e(j,i) = 1;
    plot([milestones(i,1) milestones(j,1)],[milestones(i,2) milestones(j,2)],'m');
end
disp('Time to find edges');
toc;
drawnow;

% Find shortest path
tic;
[sp, sd] = shortestpath_mr(milestones, e, 1, nM, 1, 1, 0);
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