function [spath,sdist] = shortestpath(nodes, edges, start, finish)
%SHORTESTPATH Find shortest path using A-star search
% Inputs: 
%   nodes: list of n node locations in 2D
%   edges: nXn connectivity of nodes (1 = connected), symmetric only uses
%           upper triangle
%   start: index of start node
%   finish: index of finish node


% Find edge lengths
n = length(nodes);
dists= zeros(n,n);
for i = 1:n
    for j = i:n
        if (edges(i,j))
            dists(i,j) = norm(nodes(i,:)-nodes(j,:));
            dists(j,i) = dists(i,j);
        end
    end
end

% Initialize open set (node, backtrack, lower bound cost, current cost)
dmax = norm(nodes(start,:)-nodes(finish,:));
O = [start 0 dmax 0];
% Initialize closed set (same as open set)
C = [];
done = 0;
t = 0;
% Main algorithm
while (~done)
    % Check if open set is empty
    if (length(O(:,1))==0)
        spath = [];
        sdist = 0;
        return;
    end
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
    neigh = find(edges(bestnode(1),:)==1);
    
    % Process each neighbour
    for i=1:length(neigh)
        found = find(C(:,1)==neigh(i),1);
        if (length(found)==1)
            continue;
        end
        dtogo = norm(nodes(neigh(i),:)-nodes(finish,:));
        dcur = bestnode(4)+dists(bestnode(1),neigh(i));
        found = find(O(:,1)==neigh(i),1);
        if (length(found)==0)
            O = [O; neigh(i) bestnode(1) dtogo+dcur dcur]; 
        else
            if (dcur < O(found,4))
                O(found,:) = [neigh(i) bestnode(1) dtogo+dcur dcur];
            end
        end
    end
    O = O([1:best-1 best+1:end],:);
end

% Find final path through back tracing
done = 0;
cur = finish;
curC = find(C(:,1)==finish);
prev =  C(curC,2);
spath = [cur];
sdist = 0;
while (~done)
    if (prev == start)
        done = 1;
    end
    cur = prev;
    curC = find(C(:,1)==cur);
    prev = C(curC,2);
    spath = [cur, spath];
    sdist = sdist+dists(spath(1),spath(2));
end
