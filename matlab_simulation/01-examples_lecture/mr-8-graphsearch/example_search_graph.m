%%======================================
%% GRAPH SEARCH EXAMPLE
%%--------------------------------------
% A* and Dijkstra's 
% 

close all;clear;clc;


%% Generate a graph
n = 100; % Nodes
s = 100; % Space
% Random nodes
nodes = s*rand(n,2);
e = sparse(zeros(n,n));
D = sparse(zeros*ones(n,n));

n = length(nodes);
% Calculate the distance of each pair of nodes and convert it into square
% matrix form
dists= squareform(pdist(nodes,'euclidean'));


% Add closest p edges
p = 5;
for i=1:n
    [d2,ind] = sort(dists(:,i));
    for j=1:p
        if (i~=ind(j))
            e(i,ind(j)) = 1;
            e(ind(j),i) = 1;
        end
    end
end

% Pick two points on the map with the farthest distance to be the start and
% finish for this demo.
% You can also randomly select two points if you like.
[MaxDistance,Mindex]= max(dists(:));
[finish, start] = ind2sub(size(dists),Mindex);


%% if you want to see the actual execution time on title, please turn off video. Otherwise the running speed will get slower and slower.
% SHORTESTPATH Find shortest path using differenet search algoritms
% <<Usage>>
% Inputs: 
%   1.nodes: list of n node locations in 2D
%   2.edges: nXn connectivity of nodes (1 = connected), symmetric only uses
%            upper triangle
%   3.start: index of start node
%   4.finish: index of finish node
%   5.method: (1 = Astar) (2 = Breadth-First) (3= Depth-First) (4= Dijkstra's)
%   6.heuristicDist: 1.Euclidean distance  2.Manhattan distance ...
%   7.createVideo: (1 = YES) (the others: NO) 
% Outputs:
%   1.spath: all path nodes
%   2.sdist: total distance
subplot(1,2,1)
[spath,sdist] = shortestpath_mr(nodes, e, start, finish, 4,2,0)

subplot(1,2,2)
[spath,sdist] = shortestpath_mr(nodes, e, start, finish, 1,2,0)

