function [x] = udlr_motion(map, x, v)
% This function takes in a 0-1 occupancy grid map and a robot state 
% (x y heading), and attempts to move forward one step.  If blocked, the 
% robot turns 90 degrees to the right instead.

[M,N] = size(map);
move = round(x(1:2) + v.*[cos(x(3)); sin(x(3))]);

% If the robot hits a wall or obstacle, change direction
if ((move(1)>M || move(2)>N || move(1)<1 || move(2)<1) || (map(move(1), move(2)) == 1))
    x(3) = x(3)+pi/2;
else
    x(1:2) = move;
end

return