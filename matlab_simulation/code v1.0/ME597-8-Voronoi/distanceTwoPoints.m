function [ dist ] = distanceTwoPoints( point1, point2 )
%DISTANCETWOPOINTS Returns the distance between two points
%   Detailed explanation goes here
if(point1(1) == Inf || point1(2) == Inf || point1(1) == Inf || point2(2) == Inf)
    dist = Inf;
else
    dist = sqrt((point1(1)-point2(1))^2 + (point1(2)-point2(2))^2);
end
end

