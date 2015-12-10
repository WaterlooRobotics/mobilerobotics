%% Breadth-First/Depth-First search algorithm
% Needs to be changed to unit length links, otherwise useless.
clear; clc; 

breadthfirst = 0; % 1 - breadth first, 0 - depth first

%% Create AVI object
vidObj = VideoWriter('breadthfirst.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 5;
open(vidObj);

% Get same problem every time (fix random number generator start point)
rng('default')

%% Generate a graph
n = 30; % Nodes
s = 100; % Space
% Random nodes
nodes = s*rand(n,2);
e = sparse(zeros(n,n));
D = sparse(zeros*ones(n,n));

% Add closest p edges
p = 5;
for i = 1:n
    for j = 1:n
        d(j) = norm(nodes(i,:)-nodes(j,:));
    end
    [d2,ind] = sort(d);
    for j=1:p
        if (i~=ind(j))
            e(i,ind(j)) = 1;
            e(ind(j),i) = 1;
            D(i,ind(j)) = d(ind(j));
            D(ind(j),i) = d(ind(j));
        end
    end
end

% Pick furthest apart start and end node
dmax = 0;
start = 0; finish = 0;
for i=1:n
    for j=i:n
        d = norm(nodes(i,:)-nodes(j,:));
        if (d>dmax)
            dmax = d;
            start =i;
            finish = j;
        end
    end
end

%% Plot graph
figure(1); clf; hold on;
plot(nodes(:,1),nodes(:,2),'ko');
for i = 1:n
    for j = i:n
        if (e(i,j)==1)
            plot([nodes(i,1) nodes(j,1)],[nodes(i,2) nodes(j,2)],'k');
        end
    end
end
plot(nodes(start,1),nodes(start,2),'bo','MarkerSize',6,'LineWidth',2);
plot(nodes(finish,1),nodes(finish,2),'ro','MarkerSize',6,'LineWidth',2);
writeVideo(vidObj, getframe(gcf));

%% Find shortest path

% Initialize open set (node, backtrack, lower bound cost, current cost)
O = [start 0 0];
% Initialize closed set (same as open set)
C = [];
done = 0;
t = 0;

while (~done)
    t = t+1
    % Check end condition
    if (length(O)==0)
        done = 1;
        continue;
    end

    % Grab next node in open set
    curnode = O(1,:);
    
    % Move to closed set, remove from open set
    C = [C; curnode];
    O = O([2:end],:);

    % Get all neighbours of current node
    neigh = find(e(curnode(1),:)==1);
    
    % Process each neighbour
    for i=1:length(neigh)
        % If in closed set, skip
        found = find(C(:,1)==neigh(i),1);
        if (length(found)==1)
            continue;
        end
        dcur = curnode(3)+D(curnode(1),neigh(i));
        found = find(O(:,1)==neigh(i),1);
        % If not in open set, add it
        if (length(found)==0)
            if (breadthfirst) % breadthfirst
                O = [O; neigh(i) curnode(1) dcur]; 
            else % depth first
                O = [neigh(i) curnode(1) dcur; O];
            end
        % If in open set, update cost if better    
        else
            if (dcur < O(found,3))
                O(found,:) = [neigh(i) curnode(1) dcur];
            end
        end
    end

    % Plot active nodes for this step
    figure(1);hold on;
    plot(nodes(C(:,1),1),nodes(C(:,1),2), 'ko','MarkerSize',6,'LineWidth',2);
    plot(nodes(curnode(1),1),nodes(curnode(1),2), 'go','MarkerSize',6,'LineWidth',2);
    for i=1:length(neigh)
        plot(nodes(neigh(i),1),nodes(neigh(i),2), 'mo');
        plot([nodes(curnode(1),1) nodes(neigh(i),1)],[nodes(curnode(1),2) nodes(neigh(i),2)], 'm');
    end
    writeVideo(vidObj, getframe(gcf));
end

% Find and plot final path through back tracing
done = 0;
cur = finish
curC = find(C(:,1)==finish);
prev =  C(curC,2);
i=2;
pathlength = 0;
while (~done)
    if (prev == start)
        done = 1;
    end
    figure(1);hold on;
    plot([nodes(prev,1) nodes(cur,1)], [nodes(prev,2) nodes(cur,2)],'g','LineWidth',2)
    cur = prev;
    curC = find(C(:,1)==cur);
    prev = C(curC,2);
    writeVideo(vidObj, getframe(gcf));
    pathlength = pathlength+1;
end
pathlength
close(vidObj);
