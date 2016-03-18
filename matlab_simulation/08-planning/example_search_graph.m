% Get same problem every time (fix random number generator start point)
close all;clear;
%rng('default')

%% Generate a graph
n = 50; % Nodes
s = 100; % Space
% Random nodes
nodes = s*rand(n,2);
e = sparse(zeros(n,n));
D = sparse(zeros*ones(n,n));

n = length(nodes);
dists= squareform(pdist(nodes,'euclidean'));


% Add closest p edges
p = 5;
for i = 1:n
    [d2,ind] = sort(dists(:,i));
    for j=1:p
        if (i~=ind(j))
            %count = count +1;
            e(i,ind(j)) = 1;
            e(ind(j),i) = 1;
        end
    end
end
[MaxDistance,Mindex]= max(dists(:));
[finish, start] = ind2sub(size(dists),Mindex);


%% if you want to see the actual execution time on title, please turn off video. Otherwise the running speed will get slower and slower.
subplot(2,2,1)
[spath,sdist] = shortestpath_new(nodes, e, start, finish, 1,2, 1)

subplot(2,2,2)
[spath,sdist] = shortestpath_new(nodes, e, start, finish, 2,2, 1)

subplot(2,2,3)
[spath,sdist] = shortestpath_new(nodes, e, start, finish, 3,2, 1)

subplot(2,2,4)
[spath,sdist] = shortestpath_new(nodes, e, start, finish, 4,2, 1)
