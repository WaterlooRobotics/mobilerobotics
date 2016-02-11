function traps = trapezoidalDecomposition(a,b,ptsStore,minPos,maxPos)

% the total number of obstacles
numObsts = size(b,1);

%setup the base trapezoidal.  I am assume that the "trapezoids are actually
%rectangles.  I am assuming all of the obstacles are rectangles which means
%that we can break the environment into rectangles.
baseTrap.a = zeros(4,2);
baseTrap.b = zeros(4,1);
baseTrap.pts = zeros(4,2);
baseTrap.valid = 0;
baseTrap.numEdges = 0;

baseTrap.a(1,:) = [-1 0];
baseTrap.a(2,:) = [0 1];
baseTrap.a(3,:) = [0 -1];
baseTrap.a(4,:) = [1 0];

%build the first semi-trapezoid, assuming that we are in a square environment
traps(1) = baseTrap;
traps(1).a(1,:) = [-1 0];
traps(1).a(2,:) = [0 1];
traps(1).a(3,:) = [0 -1];
traps(1).numEdges = 3;
traps(1).b(1) = traps(1).a(1,:)*[minPos(1);maxPos(2)];
traps(1).b(2) = traps(1).a(2,:)*[minPos(1);maxPos(2)];
traps(1).b(3) = traps(1).a(3,:)*[minPos(1);minPos(2)];

%find all of the x-values of each obstacle
xValues = [];
xObst = [];

for i = 1:2:2*numObsts
    len = length(unique(ptsStore(:,i)));
    xValues(end + 1:end+len,:) = unique(ptsStore(:,i));
    xObst(end+1:end+len) = ones(len,1)*((i+1)/2);
end
[xValues, xIndices] = sort(xValues);
xIndices = xObst(xIndices);

%the outward pointing normal of the vertical line used to sweep the region
aVertical = [1 0];

%current number of trapezoids
numTraps = 0;

%previous nubmer of trapezoids
oldNumTraps = 0;

numObstSides = size(a,2) ;

for i = 1:1:size(xValues)
    bVertical = xValues(i);
    obstNum = xIndices(i);

    %store the trapezoid and the yValue
    values = [];

    %the current line segments from the vertical intersecting this x-value(i)
    lineSegments = [];

    %loop through all of the edges of the obstacle and figure out if the
    %vertical which goes through x-value(i) intersects it
    for j = 1:1:size(a,1)
        if j == xIndices(i)
            continue;
        end

        %find the intersection of this line segment and the vertical line
        for k = 1:1:size(a,2)
            %if the vertical line doesnt intersect the edge, ignore it
%             if( a(j,k,2) == 0 || ~(xValues(i) >= min(ptsStore(:,2*(j-1)+1)) && xValues(i) <= max(ptsStore(:,2*(j-1)+1))))
%                 continue;
%             end
            
            if( a(j,k,2) == 0 )
                continue;
            end
            
            if( k == 1)
               if(  xValues(i) < min(ptsStore(numObstSides,2*(j-1)+1),ptsStore(k,2*(j-1)+1)) || xValues(i) > max(ptsStore(numObstSides,2*(j-1)+1),ptsStore(k,2*(j-1)+1)))
                   continue
               end
            else
                if(  xValues(i) < min(ptsStore(k-1,2*(j-1)+1),ptsStore(k,2*(j-1)+1)) || xValues(i) > max(ptsStore(k-1,2*(j-1)+1),ptsStore(k,2*(j-1)+1)))
                   continue
               end
            end

            %calculate the y-value of intersection and store it
            yValue = (aVertical(1)*b(j,k) - a(j,k,1)*bVertical)/(aVertical(1)*a(j,k,2) - aVertical(2)*a(j,k,1));
            values = [values;j,k, yValue];
        end
    end

    % get the y-values of the obstacle
    obstYValues = ptsStore(find(xValues(i) == ptsStore(:,2*(xIndices(i)-1)+1)),2*(xIndices(i)-1)+2);

    %find the correct points to make new trapezoids with from the list of
    %intersection y values with all of the edges with the current vertical
    %line
    intersectPts = [];
    if(~isempty(values))
        if(length(obstYValues) == 1)
            minPosY = obstYValues;
            maxPosY = obstYValues;
        else
            minPosY = min(obstYValues);
            maxPosY = max(obstYValues);
        end
     
        indicesX = find(values(:,3)' <= minPosY);
        pts = values(indicesX,3);
        
        if(isempty(indicesX))
            intersectPts(1) = minPos(2);
            obstEdges(1,:) = [0 0];
        else
            [val,indexX] = min( minPosY - pts);
            intersectPts(1) = values(indicesX(indexX),3);
            obstEdges(1,:) = values(indicesX(indexX),1:2);
        end

        indicesY = find(values(:,3)' >= minPosY);
        pts = values(indicesY,3);
        
        if(isempty(indicesY))
            intersectPts(2) = maxPos(2);
            obstEdges(2,:) = [0 0];
        else
            [val,indexY] = min( pts - maxPosY);

            intersectPts(2) = values(indicesY(indexY),3);
            obstEdges(2,:) = values(indicesY(indexY),1:2);
        end
    else
        % it only intersects the environment boundaries
        intersectPts = [minPos(2);maxPos(2)];
        obstEdges = [0 0;0 0];
    end

    %close existing traps
    for m = 1:1:length(obstYValues)
        traps = closeTraps(traps,[xValues(i);obstYValues(m)]);
    end

    %----------------------------------------------------------------------
    %create new traps
    if(length(obstYValues) == 1)
        if(xValues(i) == min(ptsStore(:,2*(xIndices(i)-1)+1)))
            %special case the left side
            intersectEdges = [];
            %find all of the edges that intersect that point
            for j = 1:1:size(a,2)
                if(a(obstNum,j,1)*xValues(i) + a(obstNum,j,2)*(obstYValues(1)) >= b(obstNum,j) - 100*eps && a(obstNum,j,1)*xValues(i) + a(obstNum,j,2)*(obstYValues(1)) <= b(obstNum,j) + 100*eps )
                    intersectEdges = [intersectEdges;j];
                end
            end

            %need to figure out which edge corresponds to the upper trapezoid
            %and which the bottom
            if(a(obstNum,intersectEdges(1),2) < 0)
                trapEdges(1) = 1;
                trapEdges(2) = 2;
            else
                trapEdges(1) = 2;
                trapEdges(2) = 1;
            end

            for m = 1:1:2
                traps(end+1) = baseTrap;
                traps(end).numEdges = 3;

                traps(end).a(1,:) = [-1 0];
                traps(end).b(1) = traps(end).a(1,:)*[xValues(i);obstYValues(1)];

                traps(end).a(2,:) = [-a(obstNum,intersectEdges(trapEdges(m)),1),-a(obstNum,intersectEdges(trapEdges(m)),2)];
                traps(end).b(2) = traps(end).a(2,:)*[xValues(i);obstYValues(1)];

                if(obstEdges(m,1) == 0)
                    if m == 1
                        traps(end).a(3,:) = [0 -1];
                    else
                        traps(end).a(3,:) = [0 1];
                    end
                else
                    traps(end).a(3,:) = [-a(obstEdges(m,1),obstEdges(m,2),1),-a(obstEdges(m,1),obstEdges(m,2),2)];
                end

                traps(end).b(3) = traps(end).a(3,:)*[xValues(i);intersectPts(m)];
            end
        else
            if(xValues(i) == max(ptsStore(:,2*(xIndices(i)-1)+1)))
                %special case the right side
                traps(end+1) = baseTrap;
                traps(end).numEdges = 3;

                %setup the left edge which is always vertical
                traps(end).a(1,:) = [-1 0];
                traps(end).b(1) = traps(end).a(1,:)*[xValues(i);intersectPts(1)];

                if(obstEdges(1,1) == 0)
                    traps(end).a(2,:) = [0 -1];
                else
                    traps(end).a(2,:) = [-a(obstEdges(1,1),obstEdges(1,2),1),-a(obstEdges(1,1),obstEdges(1,2),2)];
                end

                traps(end).b(2) = traps(end).a(2,:)*[xValues(i);intersectPts(1)];

                if(obstEdges(2,1) == 0)
                    traps(end).a(3,:) = [0 1];
                else
                    traps(end).a(3,:) = [-a(obstEdges(2,1),obstEdges(2,2),1),-a(obstEdges(2,1),obstEdges(2,2),2)];
                end

                traps(end).b(3) = traps(end).a(3,:)*[xValues(i);intersectPts(2)];
            else
                %special case the middle

                %figure out whether we are on the top or bottom of the
                %trapezoid
                inside = 1;
                for j = 1:1:size(a,2)
                    if(a(obstNum,j,1)*xValues(i) + a(obstNum,j,2)*(obstYValues(1)+0.01) > b(obstNum,j))
                        inside = 0;
                        break;
                    end
                end

                intersectEdges = [];
                %find all of the edges that intersect that point

                for j = 1:1:size(a,2)
                    if(a(obstNum,j,1)*xValues(i) + a(obstNum,j,2)*(obstYValues(1)) >= b(obstNum,j)-100*eps && a(obstNum,j,1)*xValues(i) + a(obstNum,j,2)*(obstYValues(1)) <= b(obstNum,j)+100*eps)
                        intersectEdges = [intersectEdges;j];
                    end
                end

                %find the right edge
                if(a(obstNum,intersectEdges(1),1) >  a(obstNum,intersectEdges(2),1))
                    edgeNum = 1;
                else
                    edgeNum = 2;
                end

                traps(end+1) = baseTrap;
                traps(end).numEdges = 3;

                %setup the left edge which is always vertical
                traps(end).a(1,:) = [-1 0];
                traps(end).b(1) = traps(end).a(1,:)*[xValues(i);obstYValues(1)];

                %setup the intersecting edge
                traps(end).a(2,:) = [-a(obstNum,intersectEdges(edgeNum),1),-a(obstNum,intersectEdges(edgeNum),2)];
                traps(end).b(2) = traps(end).a(2,:)*[xValues(i);obstYValues(1)];

                if(inside)
                    if(obstEdges(1,1) == 0)
                        traps(end).a(3,:) = [0 -1];
                    else
                        traps(end).a(3,:) = [-a(obstEdges(1,1),obstEdges(1,2),1),-a(obstEdges(1,1),obstEdges(1,2),2)];
                    end

                    traps(end).b(3) = traps(end).a(3,:)*[xValues(i);intersectPts(1)];
                else
                    if(obstEdges(2,1) == 0)
                        traps(end).a(3,:) = [0 1];
                    else
                        traps(end).a(3,:) = [-a(obstEdges(2,1),obstEdges(2,2),1),-a(obstEdges(2,1),obstEdges(2,2),2)];
                    end

                    traps(end).b(3) = traps(end).a(3,:)*[xValues(i);intersectPts(2)];
                end
            end
        end
    else
        %two y-values ... in the middle....
        
        if( xValues(i) < max(ptsStore(:,2*(xIndices(i)-1)+1)))
            %special case the middle or left
            for m = 1:1:2
                %figure out whether we are on the top or bottom of the obstacle
                inside = 1;
                for j = 1:1:size(a,2)
                    if(a(obstNum,j,1)*xValues(i) + a(obstNum,j,2)*(obstYValues(m)+0.01) > b(obstNum,j))
                        inside = 0;
                        break;
                    end
                end

                intersectEdges = [];
                %find all of the edges that intersect that point
                for j = 1:1:numObstSides
                    if(a(obstNum,j,1)*xValues(i) + a(obstNum,j,2)*(obstYValues(m)) == b(obstNum,j))
                        intersectEdges = [intersectEdges;j];
                    end
                end

                %find the correct edge
                if(a(obstNum,intersectEdges(1),1) >  a(obstNum,intersectEdges(2),1))
                    edgeNum = 1;
                else
                    edgeNum = 2;
                end

                traps(end+1) = baseTrap;
                traps(end).numEdges = 3;

                %setup the left edge which is always vertical
                traps(end).a(1,:) = [-1 0];
                traps(end).b(1) = traps(end).a(1,:)*[xValues(i);obstYValues(m)];

                %setup the intersecting edge
                traps(end).a(2,:) = [-a(obstNum,intersectEdges(edgeNum),1),-a(obstNum,intersectEdges(edgeNum),2)];
                traps(end).b(2) = traps(end).a(2,:)*[xValues(i);obstYValues(m)];

                if(inside)
                    if(obstEdges(1,1) == 0)
                        traps(end).a(3,:) = [0 -1];
                    else
                        traps(end).a(3,:) = [-a(obstEdges(1,1),obstEdges(1,2),1),-a(obstEdges(1,1),obstEdges(1,2),2)];
                    end

                    traps(end).b(3) = traps(end).a(3,:)*[xValues(i);intersectPts(1)];
                else
                    if(obstEdges(2,1) == 0)
                        traps(end).a(3,:) = [0 1];
                    else
                        traps(end).a(3,:) = [-a(obstEdges(2,1),obstEdges(2,2),1),-a(obstEdges(2,1),obstEdges(2,2),2)];
                    end

                    traps(end).b(3) = traps(end).a(3,:)*[xValues(i);intersectPts(2)];
                end
            end
        else
            traps(end+1) = baseTrap;
            traps(end).numEdges = 3;
            
            %setup the left edge which is always vertical
            traps(end).a(1,:) = [-1 0];
            traps(end).b(1) = traps(end).a(1,:)*[xValues(i);obstYValues(1)];

            if(obstEdges(1,1) == 0)
                traps(end).a(2,:) = [0 -1];
            else
                traps(end).a(2,:) = [-a(obstEdges(1,1),obstEdges(1,2),1),-a(obstEdges(1,1),obstEdges(1,2),2)];
            end

            traps(end).b(2) = traps(end).a(2,:)*[xValues(i);intersectPts(1)];

            if(obstEdges(2,1) == 0)
                traps(end).a(3,:) = [0 1];
            else
                traps(end).a(3,:) = [-a(obstEdges(2,1),obstEdges(2,2),1),-a(obstEdges(2,1),obstEdges(2,2),2)];
            end

            traps(end).b(3) = traps(end).a(3,:)*[xValues(i);intersectPts(2)];
        end
    end
    %----------------------------------------------------------------------
end

%close the last trapezoid
traps = closeTraps(traps,maxPos');

%the edges were inserted into the trapezoids out of order. Therefore we
%need to make sure they are in order in a counter-clockwise direction.
for i = 1:1:length(traps)
    if(~traps(i).valid)
        continue;
    end
    
    trapTemp = traps(i);
  
    %need to swap edge 3 and 4
    traps(i).a(3,:) = trapTemp.a(4,:);
    traps(i).b(3,:) = trapTemp.b(4,:);

    traps(i).a(4,:) = trapTemp.a(3,:);
    traps(i).b(4,:) = trapTemp.b(3,:);

    if( traps(i).a(2,2) > 0)
        %swap
        trapTemp = traps(i);

        traps(i).a(2,:) = trapTemp.a(4,:);
        traps(i).b(2,:) = trapTemp.b(4,:);

        traps(i).a(4,:) = trapTemp.a(2,:);
        traps(i).b(4,:) = trapTemp.b(2,:);
    else

    end
end


%calculate the points
traps = calculatePolyPoints(traps); 





