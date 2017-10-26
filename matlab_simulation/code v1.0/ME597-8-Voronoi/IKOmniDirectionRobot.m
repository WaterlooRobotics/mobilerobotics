function [ angular_wheel_velocity ] = IKOmniDirectionRobot( world_velocity, l, r)
%IKOMNIDIRECTIONROBOT Calculates the wheel angular velocities based on the
%required world referenced robot velocity. Assumes 
%   The robot has wheel 1 parallel to the horizontal world axis (theta=0)

theta = 0;
world_velocity = [world_velocity 0];
rot = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
robot_velocity = rot*transpose(world_velocity);
wheel_velocity = [-sin(pi/3) cos(pi/3) l; 0 -1 l; sin(pi/3) cos(pi/3) l] * robot_velocity;
angular_wheel_velocity = wheel_velocity / r;

end

