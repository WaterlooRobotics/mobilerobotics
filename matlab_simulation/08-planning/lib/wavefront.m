function [wavefrontmap, path] = wavefront(curmap, startpos, endpos, varargin)
% input: binary arrary representing the environment, 0 is object, anything greater than 0 is open space
% input: start position of the robot
% input: end position or goal location

% optional input: empty_cell value = 0
% optional input: occupied_cell value = 1

%output: wave - array with with BFS values
%output: path - vector of cells or points the robot should pass through 

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

% use a java linked list as an effiecient queue
queue = javaObject('java.util.LinkedList');

%add a occupied padding around the current map to ensure the environment is closed.
%also means you do not need to check for boundary conditions. 
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

% Initilize the que with the end point and do a BFS to label all reachable points.
queue.push([endpos(1)+1, endpos(2)+1]);
while queue.size() ~= 0 
    % pop the first entry of the que, set it to the current position
    curpos = queue.pollFirst();
    
    % current distance is the value assigned to this cell, all nodes are
    % within 1 distance of this
    current_dist = wavefrontmap(curpos(1), curpos(2));
    
    % add the test kernel, which is the 
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
