function [ collided ] = CheckCollisionSquare2( s, f, map )
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
    
    % Restrict edge intersection search to edges in target row/column that
    % are perpendicular to row/column
    a = (v-1)*2 + 1;  % index 3 or 1
    edges = map((map(:,a)==targetEdge)&(map(:,a)==map(:,a+1)),:);
    
    for i=1:size(edges,1)
        e1 = [edges(i,1) edges(i,3)];
        e2 = [edges(i,2) edges(i,4)];
        
        if((e1(u) > s(u) && e2(u) < f(u)) || ...
            (e1(u) < s(u) && e2(u) > f(u)))
            collided = 1;
            return;
        end
    end
    collided = 0;
end

