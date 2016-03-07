function [ map ] = create_a_map( M, N )
% This function creates a random map with obstacles
% M -- Height of the map
% N -- Width of the map

% Map initialization
map = zeros(M,N); 

% Boundings of map
map(1,:) = 1;
map(M,:) = 1;
map(:,1) = 1;
map(:,N) = 1;

% Obstacles
map(13:round(17+2*randn(1)),22:26) = 1;  
map(15:round(20+2*randn(1)),40:43) = 1;
map(30:34,5:7) = 1;
map(40:45,round(26+1*randn(1)):29) = 1;
map(29:30,15:32) = 1;
map(23:35,23:24) = 1;

end

