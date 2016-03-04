% Get same problem every time (fix random number generator start point)
rng('default')

%% Generate a graph
n = 30; % Nodes
s = 100; % Space
% Random nodes
nodes = s*rand(n,2);
e = sparse(zeros(n,n));
D = sparse(zeros*ones(n,n));

% Add closest p edges
p = 5;
for i = 1:n
    for j = 1:n
        d(j) = norm(nodes(i,:)-nodes(j,:));
    end
    [d2,ind] = sort(d);
    for j=1:p
        if (i~=ind(j))
            e(i,ind(j)) = 1;
            e(ind(j),i) = 1;
            D(i,ind(j)) = d(ind(j));
            D(ind(j),i) = d(ind(j));
        end
    end
end

% Pick furthest apart start and end node
dmax = 0;
start = 0; finish = 0;
for i=1:n
    for j=i:n
        d = norm(nodes(i,:)-nodes(j,:));
        if (d>dmax)
            dmax = d;
            start =i;
            finish = j;
        end
    end
end


[spath,sdist] = shortestpath(nodes, e, start, finish, 1, 1)
[spath,sdist] = shortestpath(nodes, e, start, finish, 2, 1)
[spath,sdist] = shortestpath(nodes, e, start, finish, 3, 1)
[spath,sdist] = shortestpath(nodes, e, start, finish, 4, 1)