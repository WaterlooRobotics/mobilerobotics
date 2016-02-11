function [wave,path] = wavefront(curmap,startpos, endpos)

%% Graph setup
[N,M] = size(curmap);

% Start node
starti = startpos(1);
startj = startpos(2);
start = M*(starti-1) + startj;

% End node
finish = M*(endpos(1)-1) + endpos(2);

% Node numbering and graph connectivity
e = sparse(N*M,N*M);

% For each cell in map
for i=1:N
    for j = 1:M
        % If cell is empty
        if (curmap(i,j) == 0)
            cur = M*(i-1)+j;
            % Link up if empty
            if (i>1)
                if (curmap(i-1,j) == 0)
                    e(cur, cur-M) = 1;
                    e(cur-M,cur) = 1;
                end
            end
            % Link left
            if (j>1)
                if (curmap(i,j-1) == 0)
                    e(cur, cur-1) = 1;
                    e(cur-1,cur) = 1;
                end
            end
            % Link down
            if (i<N)
                if (curmap(i+1,j) == 0)
                    e(cur, cur+M) = 1;
                    e(cur+M,cur) = 1;
                end
            end
            % Link right
            if (j<M)
                if (curmap(i,j+1) == 0)
                    e(cur, cur+1) = 1;
                    e(cur+1,cur) = 1;
                end
            end
        end % if empty
    end % j
end % i

%% Wavefront
% Essentially a breadth-first search for the cells that can be reached

% Initialize open set (node cost)
O = [finish 0];
% Initialize closed set (same form as open set)
C = [];
done = 0;
t = 1;

dist = 0;
wave = (N+M)*curmap;

while (~done)
    % Check end condition
    if (length(O)==0)
        done = 1;
        continue;
    end

    % Grab next node in open set
    curnode = O(1,:);
    
    % Move to closed set and save distance for plotting
    C = [C; curnode];
    O = O([2:end],:);
    curi = floor((curnode(1)-1)/M)+1;
    curj = mod(curnode(1),M);
    if (curj==0) curj=M; end
    wave(curi,curj) = curnode(2);

    % Get all neighbours of current node
    neigh = find(e(curnode(1),:)==1);
    
    % Process each neighbour
    for i=1:length(neigh)
        % If neighbour is already in closed list, skip it
        found = find(C(:,1)==neigh(i),1);
        if (length(found)==1)
            continue;
        end
        % If neighbour is already in open list, skip it
        found = find(O(:,1)==neigh(i),1);
        % Otherwise, add to open list at the bottom
        if (length(found)==0)
            O = [O; neigh(i) curnode(2)+1]; 
        end
    end
end

%% Shortest path

len = wave(starti,startj);
path = zeros(len,2);
path(1,:) = [starti startj];
for i=1:len
    options = [];
    if (path(i,1)>1)
        options = [options; [path(i,1)-1 path(i,2)]];
    end
    if (path(i,1)<N)
        options = [options; [path(i,1)+1 path(i,2)]];
    end
    if (path(i,2)>1)
        options = [options; [path(i,1) path(i,2)-1]];
    end
    if (path(i,2)<M)
        options = [options; [path(i,1) path(i,2)+1]];
    end
    oplen = length(options(:,1));
    for j = 1:oplen
        costs(j) = wave(options(j,1),options(j,2));
    end
    [dist, best] = min(costs);
    path(i+1,:) = options(best,:);
end

