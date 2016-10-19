function [ mup ] = simpleRobotMotionModel( state, input, dt )
% Outputs motion model from Mapping I slide 10

if ~exist('dt', 'var')
    dt = 0.1;
end

mup = [state(1) + input(1)*cos(state(3))*dt;
     state(2) + input(1)*sin(state(3))*dt;
     state(3) + input(2)*dt];


end

