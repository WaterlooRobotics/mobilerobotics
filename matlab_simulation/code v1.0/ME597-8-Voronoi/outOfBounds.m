function [ invalid ] = outOfBounds( point, upperBound )
%OUTOFBOUNDS Determines if a point is out of the space bounds
%   This is required since some of the Voronoi vertices are outside the
%   space area, and paths through these vertices should not be considered.
%   In other words, you can imagine a wall surrounding the playing space.
if (point(1,1)<0 || point(1,2)<0 || point(1,1)>upperBound || point(1,2)>upperBound)
     invalid = 1;
else invalid = 0;

end

