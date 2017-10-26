function mup = airplane_radar_motion_model(mu, ~, dt)
% Implements the motion model on slide 112 of EstimationI
% Airplane flies horizontally with a constant velocity
% mu - current state
% dt - optional timestep
% mup - predicted next state
    if ~exist('dt', 'var')
        dt = 0.1;
    end
    Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];
    mup = Ad*mu;
end