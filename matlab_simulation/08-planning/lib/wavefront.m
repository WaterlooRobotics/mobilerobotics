function [wavefrontmap, path] = wavefront(curmap, endpos, startpos)
%input: binary arrary representing the environment, 0 is object, anything greater than 0 is open space
%input: start position of the robot
%input: end position or goal location

%output: wave - array with with BFS values
%output: path - vector of cells or points the robot should pass through 

%use a java linked list as an effiecient queue
import java.util.LinkedList
queue = LinkedList();

%add a 0 padding around the current map to ensure the environment is closed.
%also means you do not need to check for boundary conditions. 
curmap = padarray(curmap,1,0);

% make an output map that is initilized to all -10, which encodes unexplored cells;
wavefrontmap = -10*ones(size(curmap));

%adjust the value of endpos to agree with the new padding. 
endpos = endpos + [1,1];

%set the end value to zero
wavefrontmap(endpos) = 0;

%set up a set of testing locations vector that encodes the point relative
%to the current position
              
test_locations = [ [0, 1];
                   [-1, 0];
                   [1, 0];                   
                   [0, -1];                  
                   ];

%Initilize the que with the end point and do a BFS to label all reachable points.
queue.push(endpos);
while queue.size() ~= 0 
    %pop the first entry of the que, set it to the current position
    curpos = queue.pollFirst();
    
    %current distance is the value assigned to this cell, all nodes are
    %within 1 distance of this
    current_dist = wavefrontmap(curpos(1), curpos(2));
    
    %add the test kernel, which is the 
    wavefrontpnts(:,1) = test_locations(:,1) + curpos(1);
    wavefrontpnts(:,2) = test_locations(:,2) + curpos(2);

    %for each of the neighboring cells check if it has been already
    %explored
    for i = 1:4
            if (wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) ~= -10)
                continue;
            end
        %if not, check that the map, the cell is not an obsticle. 
        if ((curmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) == 225))
            %assign it a distance value of the current cell + 1
            wavefrontmap(wavefrontpnts(i,1),wavefrontpnts(i,2)) = current_dist+1;
            %add this point to the que and continue looping
            queue.addLast([wavefrontpnts(i,1), wavefrontpnts(i,2)]);
        end
    end
    
end
%set unreached cells to a lower negtivive number for display purposes when
%using imagesc to show the map
wavefrontmap(wavefrontmap(:,:)==-10)= -100;

path = shortest_wavefront_path(wavefrontmap, startpos);

end


%alternative way to do what was done above. All matrices in matlab are
%actually really, really long vectors and have two index's. One is of the
%style you are used to A(1,4), but you can also reference the same point as
%just A(4). For matrices, such as a 20 x 20 matrix B, B(10, 12) is the same
%as B(20*10 + 12) = B(212). 

% curmap_size = size(curmap);
% 
%  h = int32(curmap_size(1));
%  test_kernel = int32([-h-1;
%                 -h;
%                 -h+1;
%                 -1;
%                 +1;
%                 +h-1;
%                 +h;
%                 +h+1
%                 ]);
%            
% 
% curposs = h*endpos(1) + endpos(2);
% wavefrontmap(curposs) = 0;
% queue.push(curposs);
% tic
% while queue.size() ~= 0    
%     curposs = queue.pollFirst();
%     wavefront_kernel = test_kernel + curposs;
%     current_dist = wavefrontmap(curposs);
%     wavefront_index = wavefrontmap(wavefront_kernel) == -10;
%     curmap_index = curmap(wavefront_kernel) == 225;
%     index = (wavefront_index == curmap_index) == 1;
% 
%     
%     wavefrontmap(wavefront_kernel(index)) = (current_dist + 1);
%     kern = wavefront_kernel(index);
%     for i = 1:length(kern)
%         queue.addLast(kern(i));
%     end
% end
