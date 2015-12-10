%% Visibility graph
clear all; close all; clc

% Set up environment
posMinBound = [0 0];
posMaxBound = [12 10];
numObsts = 5;
endPos = [1 1];
startPos = [21.5 18.5];

minLen.a = 1;
maxLen.a = 3;
minLen.b = 2;
maxLen.b = 6;

obstBuffer = 0.5;
maxCount = 10000;

[aObsts,bObsts,obsPtsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, numObsts, startPos, endPos, obstBuffer, maxCount);

figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos);

traps = trapezoidalDecomposition(aObsts,bObsts,obsPtsStore,posMinBound,posMaxBound);
plotTrapezoids(traps);

for i=1:length(traps)
    nodes(i,:) = (traps(i).pts(1,:)+traps(i).pts(3,:))/2;
end
for i=1:length(traps)
    for j=1:length(traps)
       if ((norm(traps(i).pts(2,:)-traps(j).pts(1,:))<0.01) || ...
           (norm(traps(i).pts(2,:)-traps(j).pts(4,:))<0.01) || ...
           (norm(traps(i).pts(3,:)-traps(j).pts(1,:))<0.01) || ...
           (norm(traps(i).pts(3,:)-traps(j).pts(4,:))<0.01))
          A(i,j) = 1;
          figure(1);hold on;
          plot([nodes(i,1) nodes(j,1)],[nodes(i,2) nodes(j,2)]) 
       end
   end
end

