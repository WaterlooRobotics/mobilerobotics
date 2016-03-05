function [ path ] = shortest_wavefront_path(wavefrontmap, starting_position)
%shortest_path: takes in a wavefront map from the wavefron function and
%returns the shortest path
%   Only one path is returned even in the case of a tie. This can be fixed
%   if you want to.

%initilize a java linked list in order to simulate a stack data structure.
import java.util.LinkedList
stack = LinkedList();


%search locations, see wavefront function for more details
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
if(wavefrontmap(curpos(1), curpos(2)) < 0)    
    error('Error. The start point is located in an obstical')
end
while wavefrontmap(curpos(1), curpos(2)) ~= 0 
    current_min = wavefrontmap(curpos(1), curpos(2))   ;
    %add the test kernel, which is the 
    wavefrontpnts(:,1) = test_locations(:,1) + curpos(1);
    wavefrontpnts(:,2) = test_locations(:,2) + curpos(2);

    %for each of the neighboring cells check if it has been already
    %explored
    for i = 1:8
            if (wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) < 0)
                continue;
            end
            %if not, check that the map, check if the next cell is lower value then the current min. 
            if ((wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) < current_min))
                best_point = [wavefrontpnts(i,1),wavefrontpnts(i,2)]; 
                current_min = wavefrontmap(curpos(1), curpos(2));
            end
    end 
    curpos = best_point;
    stack.addLast([best_point(1), best_point(2)]);
end

path = zeros(stack.size(), 2);
for i = 1:length(path)
    point = stack.removeLast();
    path(i, :) = [point(2), point(1)];
end

