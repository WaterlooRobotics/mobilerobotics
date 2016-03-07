%% distanceToLineSegment 
% A function to calculate the crosstrack error and tell whether we have
% driven past the given line segment.
%
% Input: 
% p1 - beginning point of the line segment 
% p2 - end point of the line segment
% x  - position of the vehicle
%        
% Output:
% crosstrack_error - The distance away from the line. This value can be
%                    either positive or negative depending on whether or
%                    not the robot is above or below the line segment
%
% outside - A boolean value that tells us whether or not the robot has
%           travelled the length of the line segment


function [crosstrack_error outside] = distanceToLineSegment(p1, p2, x)
outside = 0;

line_segment = p2-p1;

p1_to_x = x-p1;

% Compute the distance to the line as a vector, using the projection
projection = p1 + (dot(line_segment,p1_to_x) / norm(line_segment)^2) * line_segment;
distance = x - projection;

% If the projection falls beyond the line segment then we have driven past
% said line segment
if (dot(line_segment,p1_to_x) / norm(line_segment)^2 > 1)
    outside = 1;
end

% Take the cross product of a vector defining the line segment with a 
% vector defining the robot position. Whether it is positive or negative
% will tell us whether or not our robot is above or below the line segment.
pos_neg = 1;
cross_product = cross([line_segment 0],[p1_to_x 0]);
if(cross_product(3) < 0)
    pos_neg = -1;
end

crosstrack_error = norm(distance) * pos_neg;
