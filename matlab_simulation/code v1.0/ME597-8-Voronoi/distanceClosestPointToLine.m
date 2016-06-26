function [ distToLine ] = distanceClosestPointToLine( points, endLine1, endLine2 )
%DISTANCEPOINTTOLINE Finds the distance from the closest obstacle for a Voronoiedge 
%   A midpoint along the voronoi edge is calculated. From the array of
%   obstacles, the closest to the midpoint is found. This will also be one
%   of the two points that correspond to the Voronoi edge. The distance
%   from the obstacle to the Voronoi edge is calculated, to make sure the
%   robot can fit through.
    midpoint = [endLine1(1)+(endLine2(1)-endLine1(1))/2 endLine1(2)+(endLine2(2)-endLine1(2))/2];
    K = dsearchn(points, midpoint);
    
    a = [(endLine1 - endLine2), 0];
    b = [(points(K,:) -  endLine2), 0];
    distToLine = norm(cross(a,b))/norm(a);
end

