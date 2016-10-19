function [xddot, xd]=obstacle(T, dt, xd, xddot)
% Define the input velocity
v = 2*ones(size(T));
% Define the input angular velocity
w = zeros(size(T));
c = floor(length(w)/8);
w(2*c+1:3*c) = 1/3;
w(3*c+1:4*c) = -1/3;
w(4*c+1:5*c) = -1/3;
w(5*c+1:6*c) = 1/3;

[xddot, xd]=motion_model(T, dt, xd, xddot, v, w);
end