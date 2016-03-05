function plot_inverse_mm(og_mm, M, N, x, phi_m, r_m, fig)
% Plots the occupancy probabilities of a single measurement model timestep
% Input:
%   [og_mm] = (O)ccupancy (g)rid of the (m)easurement (m)odel
%   M = Height of map
%   N = Width of map
%   [x] = Robot state vector [x position; y position; heading]
%   [phi_m] = Array of measurement angles relative to robot
%   [r_m] = Array of (r)ange (m)easurements
%   fig = handle of figure to generate
% Output:
% 	Plot window showing occupancy window of inverse measurement model

figure(fig);
clf;
hold on;
image(100*(og_mm));
colormap(gray);
for i = 1:length(r_m)
    plot(x(2) + r_m(i)*sin(phi_m(i) + x(3)), x(1) + r_m(i)*cos(phi_m(i) + x(3)), 'rx')
end
axis([0 N 0 M])
title('Measurements and inverse measurement model');