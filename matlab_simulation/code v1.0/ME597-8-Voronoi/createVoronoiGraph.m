function [ G ] = createVoronoiGraph( C, V )
%CREATEVORONOIGRAPH Creates a Graph object our of the Voronoi diagram
%   For each set of Voronoi vertices it adds them and th edge they
%   represent to the the graph object

G = graph;
G = addnode(G,length(V));

for i=1:length(C)
    for k=1:(length(C{i})-1)
        idxOut = findedge(G,C{i}(k),C{i}(k+1));
        if(idxOut == 0)
            G = addedge(G, C{i}(k), C{i}(k+1), distanceTwoPoints(V(C{i}(k),:),V(C{i}(k+1),:)));
        end;
    end;
end;

end

