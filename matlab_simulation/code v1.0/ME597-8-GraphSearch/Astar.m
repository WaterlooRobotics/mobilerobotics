%% Dijkstra's and A-star algorithm
clear; clc; 

useAstar = 1;

%% Create AVI object
vidObj = VideoWriter('astar2.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 5;
open(vidObj);

rng('default')

%% Generate a graph
n = 100; % Nodes
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
O = [start 0 dmax 0];
% Initialize closed set (same as open set)
C = [];
done = 0;
t = 0;
while (~done)
    t = t+1
    % Find best node in open set
    [val, best] = min(O(:,3));
    bestnode = O(best,:);
    
    % Move best to closed set
    C = [C; bestnode];

    % Check end condition
    if (bestnode(1)==finish)
        done = 1;
        continue;
    end

    % Get all neighbours of best node
    neigh = find(e(bestnode(1),:)==1);
    
    % Process each neighbour
    for i=1:length(neigh)
        % If neighbour is in closed set, skip
        found = find(C(:,1)==neigh(i),1);
        if (length(found)==1)
            continue;
        end
        dtogo = norm(nodes(neigh(i),:)-nodes(finish,:));
        dcur = bestnode(4)+D(bestnode(1),neigh(i));
        found = find(O(:,1)==neigh(i),1);
        % If neighbour is not in open set, add it
        if (length(found)==0)
            if (useAstar)
                O = [O; neigh(i) bestnode(1) dtogo+dcur dcur]; 
            else % Dijkstra
                O = [O; neigh(i) bestnode(1) dcur dcur]; 
            end
        % If neighbour is in open set, check if new route is better
        else
            if (dcur < O(found,4))
                if (useAstar)
                    O(found,:) = [neigh(i) bestnode(1) dtogo+dcur dcur];
                else % Dijkstra
                    O(found,:) = [neigh(i) bestnode(1) dcur dcur];
                end
            end
        end
    end
    % remove best node from open set
    O = O([1:best-1 best+1:end],:);

    % Plot active nodes for this step
    figure(1);hold on;
    plot(nodes(C(:,1),1),nodes(C(:,1),2), 'ko','MarkerSize',6,'LineWidth',2);
    plot(nodes(bestnode(1),1),nodes(bestnode(1),2), 'go','MarkerSize',6,'LineWidth',2);
    for i=1:length(neigh)
        plot(nodes(neigh(i),1),nodes(neigh(i),2), 'mo');
        plot([nodes(bestnode(1),1) nodes(neigh(i),1)],[nodes(bestnode(1),2) nodes(neigh(i),2)], 'm');
    end
    writeVideo(vidObj, getframe(gcf));
end

% Find and plot final path through back tracing
done = 0;
cur = finish;
curC = find(C(:,1)==finish);
prev =  C(curC,2);
i=2;
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
end
Cend = find(C(:,1)==finish);
plen = C(Cend,4)

close(vidObj);
