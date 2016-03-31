function [xddot, xd]=user_define_motion_model(T, dt, xd, xddot)
% Define the input velocity
v = sin(0.2*T);
% Define the input angular velocity
w = ones(size(T));

for t=1:length(T)
    xddot(:,t) = [v(t)*cos(xd(3,t));
                v(t)*sin(xd(3,t));
                w(t)];
    xd(:,t+1) = xd(:,t)+dt*xddot(:,t);
end
end
