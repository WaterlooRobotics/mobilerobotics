function [ path ] = shortest_wavefront_path(wavefrontmap, starting_position, motion)
% shortest_path: takes in a wavefront map from the wavefront function and
% returns the shortest path

% input: wavefront_map is the matrix produced by the wavefront function
% input: starting_position: the starting location the robot for this
% iteration. If you want to change the endpoint the wavefront map must be
% updated
% input: motion is the type of allowable motion, up-down-left-right, or box
% for all 8 directions away from a current cell.
% Note that only one path is returned even in the case of a tie. 

[m,n] = size(wavefrontmap);

stack = [];

% search locations, see wavefront function for more details
switch (motion)
    case 'urdl'
        test_locations = [ [0, 1];
                   [1, 0];
                   [0, -1];                   
                   [-1, 0];                  
                   ];
    case 'box'
        test_locations = [ [0, 1];
                   [1, 1];
                   [1, 0];
                   [1, -1];
                   [0, -1];                   
                   [1, -1];                   
                   [1, 0];                   
                   [1, 1];                  
                   ];
    otherwise
        disp('No valid motion method provided')
        return;
end
        
curpos = starting_position;
best_point = [curpos(1), curpos(2)];
stack = [[best_point(1), best_point(2)]; stack]; 


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

    for i = 1:length(wavefrontpnts(:,1))
            % check if indices are valid
            if (wavefrontpnts(i,1)<= 0) || (wavefrontpnts(i,2)<= 0) || (wavefrontpnts(i,1)>= m) || (wavefrontpnts(i,2)>=n)
                continue;
            end
            
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
    stack = [[best_point(1), best_point(2)]; stack];
end

path = flipud(stack);
