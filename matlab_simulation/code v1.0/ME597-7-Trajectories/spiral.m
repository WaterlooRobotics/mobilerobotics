function [xddot, xd]=spiral(T, dt, xd, xddot)
% Define the input velocity
v = sin(0.2*T);
% Define the input angular velocity
w = ones(size(T));

[xddot, xd]=motion_model(T, dt, xd, xddot, v, w);
end