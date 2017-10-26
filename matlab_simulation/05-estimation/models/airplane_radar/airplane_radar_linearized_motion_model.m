function A = airplane_radar_linearized_motion_model(~, ~, dt)
% Implements the linearized motion model on slide 112 of EstimationI
% Airplane flies horizontally with a constant velocity
% The model is already linear, so no inputs are needed
% dt - optional timestep
% A - the A matrix of the linearized model
    if ~exist('dt', 'var')
        dt = 0.1;
    end
    Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];
    A = Ad;
end