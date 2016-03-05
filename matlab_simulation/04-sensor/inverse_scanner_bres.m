function [m] = inverse_scanner_bres(M, N, x, y, theta, r, rmax)
% Calculates the inverse measurement model for a laser scanner through
% raytracing with Bresenham's algorithm, assigns low probability of object
% on ray, high probability at end.  Returns cells and probabilities.
%
% Input:
%   M = Total Height of map
%   N = Total Width of map
%   x = Robot x position
%   y = Robot y position
%   theta = Angle of laser
%   r = Range to measured object
%   rmax = Max range of laser
% Output:
%   m = Matrix representing the inverse measurement model

% Bound the robot within the map dimensions
x1 = max(1,min(M,round(x)));
y1 = max(1,min(N,round(y)));

% Calculate position of measured object (endpoint of ray)
endpt = [x y] + r*[cos(theta) sin(theta)];

% Bound the endpoint within the map dimensions
x2 = max(1,min(M,round(endpt(1))));
y2 = max(1,min(N,round(endpt(2))));

% Get coordinates of all cells traversed by laser ray 
[list(:,1), list(:,2)] = bresenham(x1,y1,x2,y2);

% Assign probabilities
m = [list 0.4*ones(length(list(:, 1)),1)];
if (r<rmax)
    m(end,3) = 0.6;
end


