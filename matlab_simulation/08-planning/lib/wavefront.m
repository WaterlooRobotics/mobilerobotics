function [wavefrontmap, path] = wavefront(curmap, startpos, endpos, varargin)
% input: binary arrary representing the environment, 0 is object, anything greater than 0 is open space
% input: start position of the robot
% input: end position or goal location

% optional input: empty_cell value = 0
% optional input: occupied_cell value = 1

% output: wavefrontmap - array with the resulting wavefront map
% output: path - vector of points that contain the shortest path between
% the start and end goals

% if the user has differnt values for occupied and unoccupied cells check
% that they have provided 2 arguments, and then assign the arguments
% accordingly
if ~isempty(varargin) > 0
    if length(varargin) == 2
        empty_cell = varargin{1};
        occupied_cell = varargin{2};
    else
        error('Empty and occupied cell values must be specified together')
    end
else
    empty_cell = 0;
    occupied_cell = 1;
end

% The wavefront planner is essentially a Breadth First Search (BFS) over
% the entire map. An ideal data struture to use for this problem is a queue
% which can be easily imported into matlab by using Java's linked list. You
% can actually import a number of data strutures from Java that greatly
% improve Matlab's performance at some tasks.
queue = javaObject('java.util.LinkedList');

% To save on checks during the breadth first search, I can avoid checking
% for boundardy conditions (ie checking that index of the cell I am
% searching for is located within the map) by padding the cell with a layer
% of obstical cells. 
curmap = padarray(curmap, 1, occupied_cell);

% make an output map that is initilized to all -10, which encodes unexplored cells;
wavefrontmap = -10*ones(size(curmap));

% adjust the value of stard and endpos to agree with the new padding. 
% set the end value to zero
wavefrontmap((endpos(1)+1), (endpos(2)+1)) = 0;

% set up a set of testing locations vector that encodes the point relative
% to the current position
              
test_locations = [ [0, 1];
                   [-1, 0];
                   [1, 0];                   
                   [0, -1];                  
                   ];

% Initilize the queue with the end point and do a BFS to label all reachable points.
queue.push([endpos(1)+1, endpos(2)+1]);
while queue.size() ~= 0 
    % pop the first entry of the que, set it to the current position
    curpos = queue.pollFirst();
    
    % current distance is the value assigned to this cell, all nodes are
    % within 1 distance of this
    current_dist = wavefrontmap(curpos(1), curpos(2));
    
    % update the vector of test points for the current cell;
    wavefrontpnts(:,1) = test_locations(:,1) + curpos(1);
    wavefrontpnts(:,2) = test_locations(:,2) + curpos(2);

    % for each of the neighboring cells check if it has been already
    % explored
    for i = 1:4
            if (wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) ~= -10)
                continue;
            end
        % if not, check that the map, the cell is not an obsticle. 
        if ((curmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) == empty_cell))
            % assign it a distance value of the current cell + 1
            wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) = current_dist+1;
            % add this point to the que and continue looping
            queue.addLast([wavefrontpnts(i,1), wavefrontpnts(i,2)]);
        end
    end    
end
% set unreached cells to a lower negtivive number for display purposes when
% using imagesc to show the map
wavefrontmap(wavefrontmap(:,:)==-10)= -100;

%remove the padding
wavefrontmap = wavefrontmap(2:end-1, 2:end-1);
path = shortest_wavefront_path(wavefrontmap, startpos);

end
