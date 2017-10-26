function [ mode ] = collision_pred( t, X, x_vel, y_vel, map)
% This function can judge whether the robot will hit the map boundaries or
% obstacles and returns a strategy (the output "mode") to avoid the
% collision.

% Encode each hitting situation into a binary code [a,b,c,d], in which 
% the element "a" represents the coordinate to the left of the predicted robot position,
% the element "b" represents the coordinate above the predicted robot position,
% the element "c" represents the coordinate to the right of the predicted robot position,
% the element "d" represents the coordinate under of the predictied robot position,

% Size of the real map
 [M N] = size(map);

% Predition of robot positions
 pred_x = X(1,t-1) + x_vel; % X-axis position at next time step (t)
 pred_y = X(2,t-1) + y_vel; % Y-axis position at next time step (t)
    
% Combination of comparision vector (coordinate)
    if ((pred_x == 1) || (pred_x == N)) % If the robot hit the vertical boundaries
        if x_vel<0
          coordinate = [ 1, map( pred_x,pred_y+1), map( pred_x+1, pred_y), map( pred_x, pred_y-1) ]; % Left boundary
        else x_vel>0
          coordinate = [ map( pred_x-1,pred_y), map( pred_x,pred_y+1), 1, map( pred_x, pred_y-1) ]; % Right boundary
        end
    elseif ((pred_y == 1) || (pred_y == M)) % If the robot hit the horizontal boundaries
        if y_vel>0
          coordinate = [ map( pred_x-1,pred_y), 1, map( pred_x+1, pred_y), map( pred_x, pred_y-1) ]; % Upper boundary
        else y_vel<0
          coordinate = [ map( pred_x-1,pred_y), map( pred_x,pred_y+1), map( pred_x+1, pred_y), 1 ]; % lower boundary
        end
    else
          coordinate = [ map( pred_x-1, pred_y), map( pred_x,pred_y+1), map( pred_x+1, pred_y), map( pred_x, pred_y-1) ]; % Not going to hit map boundaries
    end
            
    % Corresponding values of coordinate when hitting map boundary or
    % obstacles
    
    % hit rightwards
    r1 = [0, 1, 1, 1];
    r2 = [0, 0, 1, 1];
    r3 = [0, 1, 1, 0];
    
    % hit downwards
    d1 = [1, 0, 1, 1];
    d2 = [0, 0, 1, 1];
    d3 = [1, 0, 0, 1];
    
    % hit upwards
    u1 = [1, 1, 1, 0];
    u2 = [0, 1, 1, 0];
    u3 = [1, 1, 0, 0];
    
    % hit leftwards
    l1 = [1, 1, 0, 1];
    l2 = [1, 0, 0, 1];
    l3 = [1, 1, 0, 0];
    
            if  (x_vel > 0) && ( isequal(coordinate,r1) || isequal(coordinate,r2) || isequal(coordinate,r3)) % hit rightwards
                mode = 1;
            elseif (y_vel < 0) && (isequal(coordinate,d1) || isequal(coordinate,d2) || isequal(coordinate,d3)) % hit downwards
                mode = 2;
            elseif (y_vel > 0) && ( isequal(coordinate,u1) || isequal(coordinate,u2) || isequal(coordinate,u3) ) % hit upwards
                mode = 3;
            elseif (x_vel < 0) && ( isequal(coordinate,l1) || isequal(coordinate,l2) || isequal(coordinate,l3) ) % hit leftwards
                mode = 4;
            else
                mode = 5;
            end
end
