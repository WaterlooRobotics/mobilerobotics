function [X] = robot_motion( map, X, x0, T)

% This function create a random robot position and a random robot motion
% map -- real map
% X -- robot states
% x0 -- initial robot state
% T -- time of simulation

%% Randomly generate robot x & y velocities
 
 x_vel = 1; % Initial x velocity
 y_vel = 1; % Initial y velocity
 theta_vel = 0.2; % Spinning rate
 
 while (x_vel * y_vel == 0)
    x_vel = 2*round(0.4*randn(1));
    y_vel = 2*round(0.4*randn(1));
 end

 [M N] = size(map);

 X(:,1) = x0; % Initial state of the robot
 
 %%
    mode = 5; % Robot initial moving mode
 for t = 2:T+1
     
    mode = collision_pred( t, X, x_vel, y_vel, map ); % Judge which mode should be used at current loop
     
    switch (mode)  
        case 1 % hit rightwards
            if y_vel > 0 
                x_vel = -1;
                y_vel = 1;
            
              elseif y_vel < 0
                x_vel = -1;
                y_vel = -1;
              else
                 x_vel = - x_vel;
            end
            
             X(1,t) =  X(1,t-1) + x_vel;
             X(2,t) =  X(2,t-1) + y_vel;
             X(3,t) = X(3,t-1) + theta_vel;
             mode = 5;
             
        case 2 % hit downwards
           if x_vel > 0 
                x_vel = 1;
                y_vel = 1;
            
              elseif x_vel < 0
                x_vel = -1;
                y_vel = 1;
              else
                 y_vel = - y_vel;
           end
           
             X(1,t) =  X(1,t-1) + x_vel;
             X(2,t) =  X(2,t-1) + y_vel;
             X(3,t) = X(3,t-1) + theta_vel;
             mode = 5;
             
        case 3 % hit upwards
           if x_vel > 0 
                x_vel = 1;
                y_vel = -1;
            
              elseif x_vel < 0
                x_vel = -1;
                y_vel = -1;
              else
                 y_vel = - y_vel;
           end
           
             X(1,t) =  X(1,t-1) + x_vel;
             X(2,t) =  X(2,t-1) + y_vel;
             X(3,t) = X(3,t-1) + theta_vel;
             mode = 5;
             
        case 4 % hit leftwards
           if y_vel > 0 
                x_vel = 1;
                y_vel = 1;
            
              elseif y_vel < 0
                x_vel = 1;
                y_vel = -1;
              else
                 x_vel = - x_vel;
           end
           
             X(1,t) =  X(1,t-1) + x_vel;
             X(2,t) =  X(2,t-1) + y_vel;
             X(3,t) = X(3,t-1) + theta_vel;
             mode = 5;
    
        case 5 % travel at the current speed
            
             X(1,t) =  X(1,t-1) + x_vel;
             X(2,t) =  X(2,t-1) + y_vel;
             X(3,t) = X(3,t-1) + theta_vel;
             mode = 5;            
    end
    
end

