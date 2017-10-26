function plot_cell_map(map, M, N, fig)
% Plots a cell map in black and white
% Input:
%   [map] = Cell map in matrix form
%   M = Height of map
%   N = Width of map
%   fig = handle of figure to generate
% Output:
% 	Plot window showing cell map in black and white

figure(fig);
clf; 
hold on;
image(100 * (1 - map));
colormap(gray);
axis([0 N 0 M])
title('Cell Map');