%% Voronoi Diagram
clear; clc; 

%% Create AVI object
vidObj = VideoWriter('voronoi.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 5;
open(vidObj);


%% Generate a set of point obstacles
n = 300; % Points
s = 1000; % Space

%rng('default')

% Random nodes
points = s*rand(n,2);
figure(1);clf;
plot(points);
dt = delaunayTriangulation(points(:,1),points(:,2));
H = voronoi(dt);
[V,C] = voronoiDiagram(dt);
writeVideo(vidObj, getframe(gcf));

% Generate start and end locations
startLocation = s*rand(1,2);
endLocation = s*rand(1,2);
%Closes Voronoi vertex to the end and start locations
closestToStart = dsearchn(V, startLocation);
closestToEnd = dsearchn(V, endLocation);

writeVideo(vidObj, getframe(gcf));
hold on;

plot(V(closestToStart,1),V(closestToStart,2),'ro','MarkerSize',6,'LineWidth',2);
writeVideo(vidObj, getframe(gcf));

plot(V(closestToEnd,1),V(closestToEnd,2),'ro','MarkerSize',6,'LineWidth',2);
writeVideo(vidObj, getframe(gcf));

%% Initialize Robot
l = 10;
r = 5;
robotWidth = 2*l;
wheelVelocity = [];

%% Create graph with edges of the Voronoi Diagram
G = graph;
G = addnode(G,length(V));

for i=1:length(C)
    for k=1:(length(C{i})-1)
        idxOut = findedge(G,C{i}(k),C{i}(k+1));
        if(idxOut == 0)
            G = addedge(G, C{i}(k), C{i}(k+1), distanceTwoPoints(V(C{i}(k),:),V(C{i}(k+1),:)));    
        end; 
    end;
end;

%% Find shortest path
% Discard any paths passing in between obtacles where the robot wouldn't
% fit. When this happens, remove the edge from the graph and recalculate
% the shortest path.
path = shortestpath(G, closestToStart, closestToEnd);
i=1;
pathLineObjects = [];
while i<=(length(path)-1)
    distBetweenObstacles = 2*distanceClosestPointToLine(points, V(path(i),:), V(path(i+1),:));
    if(distBetweenObstacles < robotWidth || outOfBounds(V(path(i),:),s) || outOfBounds(V(path(i+1),:),s))
        display('Have to find another path');
        erasePath(pathLineObjects);
        G = rmedge(G, path(i), path(i+1));
        i = 1;
        pathLineObjects = [];
        path = shortestpath(G, closestToStart, closestToEnd);
        writeVideo(vidObj, getframe(gcf));
    else
        pathLineObj = line([V(path(i),1) V(path(i+1),1)], [V(path(i),2) V(path(i+1),2)],'Color',[1,0,0]); 
        pathLineObjects = [pathLineObjects, pathLineObj];
        i = i+1;
        writeVideo(vidObj, getframe(gcf));
    end;
end;

%% Calculate angular velocity of wheels
% A different wheel veloicty is calculated for each path segment. 
i=1;
wheelVelocity = [];
while i<=(length(path)-1)
    worldVelocity = V(path(i+1),:) - V(path(i),:);
    wheelVelocity = [wheelVelocity; transpose(IKOmniDirectionRobot(worldVelocity, l, r))];
    i = i+1;
end;

close(vidObj);