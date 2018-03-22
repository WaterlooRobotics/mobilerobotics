function [ collided ] = CheckCollisionSquare( s, f, map )
    v = 0;
    u = 0;
    if(s(1) == f(1))
        % Find edge going right
        v = 2;
        u = 1;
    else
        % Find edge going up
        v = 1;
        u = 2;
    end
    targetEdge = (s(v)+f(v))/2;
    
    for i=1:size(map,1)
        e1 = [map(i,1) map(i,3)];
        e2 = [map(i,2) map(i,4)];
        
        if(e1(v) == e2(v) && e1(v) == targetEdge)
            if((e1(u) > s(u) && e2(u) < f(u)) || ...
                (e1(u) < s(u) && e2(u) > f(u)))
                collided = 1;
                return;
            end
        end
    end
    collided = 0;
end

