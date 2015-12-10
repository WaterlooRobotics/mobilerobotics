%% Voronoi Diagram

%% Generate a set of point obstacles
n = 100; % Points
s = 100; % Space

% Random nodes
points = s*rand(n,2);
figure(1);clf;
voronoi(points(:,1),points(:,2));

