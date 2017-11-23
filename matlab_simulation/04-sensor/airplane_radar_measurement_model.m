function yp = airplane_radar_measurement_model(mup, ~)
% Implements the measurement model on slide 113 of EstimationI
% The radar returns the straight-line distance from a point on the ground
% to the airplane position
% mup - predicted state
% yp - predicted measurement
    yp = sqrt(mup(1)^2 + mup(3)^2);
end