function [xddot, xd]=motion_model(T, dt, xd, xddot, v, w)
% Define the states
% xd1: x-position; xd2: y-position; xd3: heading angle
% xddot1: x-velocity; xddot2: y-velocity; xddot3: angular velocity
for t=1:length(T)
    xddot(:,t) = [v(t)*cos(xd(3,t));
                  v(t)*sin(xd(3,t));
                  w(t)];
    xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
end
end