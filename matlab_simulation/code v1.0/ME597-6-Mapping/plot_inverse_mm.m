function plot_inverse_mm(og_mm, M, N, x, r_m, phi_m, fig)
% Plots the occupancy probabilities of a single measurement model timestep
% Input:
%   [og_mm] = (O)ccupancy (g)rid of the (m)easurement (m)odel
%   M = Height of map
%   N = Width of map
%   [x] = Robot state vector [x position; y position; heading]
%   [r_m] = Array of range measurements
%   [phi_m] = Array of measurement angles relative to robot
%   fig = handle of figure to generate
% Output:
% 	Plot window showing occupancy grid 

figure(fig);
clf;
hold on;

image(100*(og_mm));
colormap(gray);
for i=1:length(r_m)
%    plot(x(2) + r_m(i)*sin(phi_m(i) + x(3)), x(1)+r_m(i)*cos(phi_m(i) + x(3)), 'ko')
end
axis([0 N 0 M])
%F2(t-1) = getframe;
title('Measurements and inverse measurement model');
