function [ A ] = simpleLinearizedRobotMotionModel( mu, input, dt )
% Outputs linearized motion model from Mapping I slide 15

if ~exist('dt', 'var')
    dt = 0.1;
end

A = [ 1 0 -input(1)*sin(mu(3))*dt;
      0 1 input(1)*cos(mu(3))*dt;
      0 0 1];

end
