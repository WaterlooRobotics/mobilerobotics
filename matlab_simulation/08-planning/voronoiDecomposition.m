function [path_coord,path_length] = voronoiDecomposition(start, finish, obstacles, plot_path)
%VORONOIDECOMPOSITION Plan path around obstacles using Voronoi diagrams.
% Inputs: 
%   start: Start position in 2D [x,y]
%   end: End position in 2D [x,y]
%   obstacles: List of n node locations in 2D [x1,y1;x2,y2;x3,y3...]
%   plot: boolean value (a value of true produces a plot of the motion)
% Outputs:
%   path_coord: List of node locations from start to finish in 
%               2D [x1,y1;x2,y2;x3,y3...
%   path_length: Returns length of path

%Generate Voronoi graph
[vx,vy] = voronoi(obstacles(:,1),obstacles(:,2));

%Calculate area of interest
x_max = max(obstacles(:,1));
x_min = min(obstacles(:,1));
y_max = max(obstacles(:,2));
y_min = min(obstacles(:,2));

%Remove points outside of environment
id = (vx(1,:) < x_min) | (vx(2,:) < x_min);
vx(:,id) = [];
vy(:,id) = [];
id = (vx(1,:) > x_max) | (vx(2,:) > x_max);
vx(:,id) = [];
vy(:,id) = [];
id = (vy(1,:) < y_min) | (vy(2,:) < y_min);
vx(:,id) = [];
vy(:,id) = [];
id = (vy(1,:) > y_max) | (vy(2,:) > y_max);
vx(:,id) = [];
vy(:,id) = [];

vx_pnts = vx(:);
vy_pnts = vy(:);

%Find closest point to start
dist = zeros(2,length(vx_pnts));
for i = 1:length(vx_pnts)
    dist(1,i) = sqrt((vx_pnts(i)-start(1))^2+(vy_pnts(i)-start(2))^2);
    dist(2,i) = sqrt((vx_pnts(i)-finish(1))^2+(vy_pnts(i)-finish(2))^2);
end

%Add extra edges to connect start and end points to graph
[~,ind] = min(dist(1,:));
vx = [vx [start(1); vx_pnts(ind)]];
vy = [vy [start(2); vy_pnts(ind)]];
closestto_start = [vx_pnts(ind(1)),vy_pnts(ind(1))];

[~,ind] = min(dist(2,:));
closestto_end = [vx_pnts(ind(1)),vy_pnts(ind(1))];
vx = [vx [finish(1); vx_pnts(ind)]];
vy = [vy [finish(2); vy_pnts(ind)]];

%Create edges matrix representing connections between nodes (1=connected)
nodes = unique([vx(:),vy(:)],'rows');
edges = zeros(length(nodes));
pnt_match = zeros(length(vx),2);
for i=1:length(vx)
    pnt1 = [vx(1,i),vy(1,i)];
    pnt2 = [vx(2,i),vy(2,i)];
    for j=1:length(nodes)
        if isequal(nodes(j,:),pnt1)
            pnt_match(i,1) = j;
        end
        if isequal(nodes(j,:),pnt2)
            pnt_match(i,2) = j;
        end
        if isequal(nodes(j,:),start) && (isequal(pnt1,start) || isequal(pnt2,start))
            start_id = j;
        elseif isequal(nodes(j,:),finish) && (isequal(pnt1,finish) || isequal(pnt2,finish))
            finish_id = j;
        end
    end
    pnt_match(i,:) = sort(pnt_match(i,:));
    edges(pnt_match(i,1),pnt_match(i,2))=true;
end

%Calculate shortest path along graph using existing function
[spath,path_length] = shortestpath(nodes,edges,start_id,finish_id);
path_coord = [nodes(spath,1),nodes(spath,2)];

%Plotting to show veronoi structure, obstacles & path
if plot_path
    figure
    hold on
    scatter(obstacles(:,1),obstacles(:,2),'g')
    plot(nodes(spath,1),nodes(spath,2),'--b')
    plot(start(1),start(2),'b+',finish(1),finish(2),'r+');
    text(nodes(start_id,1),nodes(start_id,2),'start')
    text(nodes(finish_id,1),nodes(finish_id,2),'finish')
    % When uncommented, this section will plot the Veronoi edges and nodes
    % plot(vx,vy,'b-')
    % for j=1:length(nodes)
    %     label = sprintf('%d',j);
    %     text(nodes(j,1),nodes(j,2),label)
    % end
    axis([x_min,x_max,y_min,y_max]);
    axis square
end
end





