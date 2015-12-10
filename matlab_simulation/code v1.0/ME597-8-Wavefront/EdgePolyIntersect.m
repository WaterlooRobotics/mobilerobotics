function [ intersect, intEdge ] = EdgePolyIntersect( edge, poly )
%EDGEPOLYINTERSECT Checks if a line interesects with a polygon
%  Looks for an intersection of the edge with each edge of the polygon.
%  Edge must be defined as a 2x2 matrix where each row is a vertex.
%  Polygon is defined as a nX2 matrix where each row is a vertex.

% Convert poly from [x1 y1; x2 y2; x3 y3;...] format to edge format
polyE = [poly [poly(2:end,:); poly(1,:)]];
% Convert edge from [x1 y1; x2 y2] format to edge format
edgeE = [edge(1,:) edge(2,:)];
   
% Check for each edge of polygon
intersect = 0;
intEdge = [];

% Find min/max x,y extend of edge
exmin = min(edge(:,1));
exmax = max(edge(:,1));
eymin = min(edge(:,2));
eymax = max(edge(:,2));

for k = 1:size(polyE,1)
    % Find min/max x,y extend of polygon edge
    pxmin = min(poly(:,1));
    pxmax = max(poly(:,1));
    pymin = min(poly(:,2));
    pymax = max(poly(:,2));
    
    % If the edge isn't outside the bounding box that defines the polygon
    if ~((pxmin > exmax) || (pxmax < exmin) || (pymin > eymax) || (pymax < eymin))
        % Check for a collision
        if (EdgeCollision(edgeE, polyE(k,:)))
            % Eliminate end-to-end contacts from collisions list
            if (sum(abs(edgeE(1,1:2)-polyE(k,1:2)))>0 && ...
                sum(abs(edgeE(1,3:4)-polyE(k,1:2)))>0 && ...
                sum(abs(edgeE(1,1:2)-polyE(k,3:4)))>0 && ...
                sum(abs(edgeE(1,3:4)-polyE(k,3:4)))>0)
            
                intEdge = k;
                intersect = 1 ; % Edge and polygon intersect
                return
            end
        end
    end
end
