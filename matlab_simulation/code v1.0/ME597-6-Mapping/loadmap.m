function [map]=loadmap(mapNum)


M = 50;
N = 60;

map = zeros(M,N);
map(4:10,5:10) = 1;
map(30:35,40:45) = 1;
map(3:6,40:60) = 1;
map(20:30,25:29) = 1;
map(40:50,5:25) = 1;


