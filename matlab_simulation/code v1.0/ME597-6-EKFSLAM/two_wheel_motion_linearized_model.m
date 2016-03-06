function Gt = two_wheel_motion_linearized_model(x,u,dt)
% linearized motion model of a 2-wheel robot
% can be found P.20 of Mapping II slides
if nargin == 2
        dt = 0.1;
end
    Gt = [ 1 0 -u(1)*sin(x(3))*dt;
           0 1 u(1)*cos(x(3))*dt;
           0 0 1];
end
