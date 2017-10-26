function [xddot, xd]=corner(T, dt, xd, xddot)
% Define the input velocity
v = 2*ones(size(T));
% Define the input angular velocity
w = zeros(size(T));
c = floor(length(w)/8);
w(2*c+1:5*c) = -2/5;

[xddot, xd]=motion_model(T, dt, xd, xddot, v, w);
end