function plot_occupancy_grid(og, M, N, fig)
% Plots an occupancy grid with belief probabilities shown in grayscale
% Input:
%   [og] = Existing occupancy grid in log odds form
%   M = Height of map
%   N = Width of map
%   fig = handle of figure to generate
% Output:
% 	Plot window showing occupancy grid 

% Belief map
figure(fig);
clf; 
hold on;
image(100*(og));
colormap(gray);
%plot(x(2,max(1,t-10):t),x(1,max(1,t-10):t),'bx-')
axis([0 N 0 M])
%F3(t-1) = getframe;
title('Current occupancy grid map')
%if (makemovie2) writeVideo(vidObj2, getframe(gca)); end