function [map, M, N, x] = load_cell_map(mapNum)
% Creates and returns a 2D cell map used for robotic simulations
%
% Input:
% 	mapNum = Number of map to return
% 		1 = Default map, smaller map with mostly empty space and some blocks, 
%			size: (50 x 60)
% 		2 = Map with lots of scatttered obstacles, size: (100 x 100)
%		3 = Massive map to test large environments, size (1000 x 1000)
% Output:
% 	[map] = The map in matrix form where 1 represents and occupied cell and
%		0 represents an empty cell
%   M = The height of the map
%   N = The width of the map
%	[x] = The inital state of the robot in vector form

% Map features to be used in different size maps
submap = zeros(50, 50);
submap(20:22, 10:12) = 1;
submap(48:50, 48:50) = 1;
submap(10:14, 35:37) = 1;
submap(30:33, 20:26) = 1;

switch (mapNum)
	case 1		
		% Map size and contents
		M = 50;
		N = 60;
		map = zeros(M, N);
		map(4:10, 5:10) = 1;
		map(30:35, 40:45) = 1;
		map(3:6, 40:60) = 1;
		map(20:30, 25:29) = 1;
		map(40:50, 5:25) = 1;
		
		% Initial Robot location
		x = [25; 10; 0];
	case 2
		% Map size and contents
		M = 100;
		N = 100;
		map = zeros(M, N);
		map(1:50, 1:50) = submap;
		map(51:100, 1:50) = submap;
		map(1:50, 51:100) = submap;
		map(51:100, 51:100) = submap;
		
		% Initial Robot location
		x = [25; 10; 1.5];
    case 3
		% Map size and contents
		M = 1000;
		N = 1000;
		map = zeros(M, N);
        for i = 1:M/20:M
            for j = 1:N/20:N
                map(i:(i + 49), j:(j + 49)) = submap;
            end
        end
        
		% Initial Robot location
		x = [500; 800; 0];
	otherwise
		% Default to case 0
		% Map size and contents
		M = 50;
		N = 60;
		map = zeros(M, N);
		map(4:10, 5:10) = 1;
		map(30:35, 40:45) = 1;
		map(3:6, 40:60) = 1;
		map(20:30, 25:29) = 1;
		map(40:50, 5:25) = 1;
		
		% Initial Robot location
		x = [25; 10; 0];
end
