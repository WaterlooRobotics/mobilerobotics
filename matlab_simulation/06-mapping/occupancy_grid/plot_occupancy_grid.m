function plot_occupancy_grid(og, M, N, fig)
% Plots an occupancy grid with belief probabilities shown in grayscale
% Input:
%   [og] = Existing occupancy grid in log odds form
%   M = Height of map
%   N = Width of map
%   fig = handle of figure to generate
% Output:
% 	Plot window showing occupancy grid 

figure(fig);
clf; 
hold on;
image(100 * (og));
colormap(gray);
axis([0 N 0 M])
title('Current occupancy grid map')