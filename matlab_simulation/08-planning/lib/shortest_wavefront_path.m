function [ path ] = shortest_wavefront_path(wavefrontmap, starting_position)
% shortest_path: takes in a wavefront map from the wavefront function and
% returns the shortest path

% input: wavefront_map is the matrix produced by the wavefront function
% input: starting_position: the starting location the robot for this
% iteration. If you want to change the endpoint the wavefront map must be
% updated

% Note that only one path is returned even in the case of a tie. 

% initilize a java linked list in order to simulate a stack data structure.
stack = javaObject('java.util.LinkedList');


% search locations, see wavefront function for more details
test_locations = [[-1, 1];
                   [0, 1];
                   [1, 1];
                   [-1, 0];
                   [1, 0];
                   [-1, -1];
                   [0, -1];
                   [-1, -1];
                   ];
curpos = starting_position;
best_point = [curpos(1), curpos(2)];
stack.addLast([best_point(1), best_point(2)]) 


if(wavefrontmap(curpos(1), curpos(2)) < 0)    
    error('Error. The start point is located in an obstical or the endpoint is unreachable')
end

while wavefrontmap(curpos(1), curpos(2)) ~= 0 
    current_min = wavefrontmap(curpos(1), curpos(2))   ;
    % create a vector of points relative to the rurrent posistion. 
    % This vector contains reference to the cells located above, below,
    % left and right of the current location (curpos)
    wavefrontpnts(:,1) = test_locations(:,1) + curpos(1);
    wavefrontpnts(:,2) = test_locations(:,2) + curpos(2);


    for i = 1:8
            % check to see if cell if free
            if (wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) < 0)
                continue;
            end
            % if a cell is free check its total cost is less than all other
            % moves at this point. (Recall we are moving down the gradient
            % to reach the end goal. 
            if ((wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) < current_min))
                best_point = [wavefrontpnts(i,1),wavefrontpnts(i,2)]; 
                current_min = wavefrontmap(curpos(1), curpos(2));
            end
    end 
    curpos = best_point;
    % Save the best point by pushing it to the stack
    stack.addLast([best_point(1), best_point(2)]);
end

path = zeros(stack.size(), 2);
% Pop each value from the stack to return the shortest path.
for i = 1:length(path)
    point = stack.removeLast();
    path(i, :) = [point(2), point(1)];
end
