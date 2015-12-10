% iterates over all of the trapezoids that are not completed, and figures
% out if the point passed in is inside the trapezoid.  If so, it completes
% the trapezoid and closes it off.  Assumption that the last edge is
% vertical. (vertical decomposition).

function traps = closeTraps(traps, point)

for j = 1:1:length(traps)
    if(~traps(j).valid)
        count = 0;
        for k = 1:1:traps(j).numEdges
            if( traps(j).a(k,:)*point - traps(j).b(k) <= 100*eps)
                count = count + 1;
            end
        end

        %the point is inside the trapezoid
        if(count == size(traps(j).a,1)-1)
            traps(j).a(end,:) = [1;0];
            traps(j).b(end) = traps(j).a(end,:)*point;
            traps(j).valid = 1;
            traps(j).numEdges = traps(j).numEdges + 1;
        end
    end
end