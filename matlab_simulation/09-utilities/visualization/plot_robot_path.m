function plot_robot_path(x, t, fig)
% Plots a robot path on a given figure handle
% Input:
%   [x] = Matrix of robot state vectors, time is the other dimension 
%   [t] = Current simulation time index
%   fig = handle of figure to generate
% Output:
% 	Plot window showing robot path

figure(fig);
plot(x(2, 1:t), x(1, 1:t), 'bx-')