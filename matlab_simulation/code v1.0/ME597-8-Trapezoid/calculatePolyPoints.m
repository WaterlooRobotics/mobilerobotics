function polygons = calculatePolyPoints(polygons)
%calculates the points of a polygon.  It assumes that the edges are in
%order (meaning if start clockwise or counterclockwise the edges are in
%order that you would reach them)

for i = 1:1:length(polygons)
    if(~polygons(i).valid)
        continue;
    end

    numEdges = size(polygons(i).a,1);
    nextEdge = [2:1:numEdges, 1];
    for j = 1:1:numEdges
        
        m = nextEdge(j);
%         if j == numEdges
%             m = 1;
%         else
%             m = j + 1;
%         end

        pt = [-(polygons(i).a(m,2)*polygons(i).b(j) - polygons(i).b(m)*polygons(i).a(j,2))/(polygons(i).a(m,1)*polygons(i).a(j,2) - polygons(i).a(j,1)*polygons(i).a(m,2));...
            (polygons(i).a(m,1)*polygons(i).b(j) - polygons(i).a(j,1)*polygons(i).b(m))/(polygons(i).a(m,1)*polygons(i).a(j,2) - polygons(i).a(m,2)*polygons(i).a(j,1))];
        
        polygons(i).pts(j,:) = pt;
    end
end