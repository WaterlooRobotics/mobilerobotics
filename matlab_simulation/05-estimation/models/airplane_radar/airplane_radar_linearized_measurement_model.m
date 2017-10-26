function H = airplane_radar_linearized_measurement_model(mup, ~)
% Implements the measurement model on slide 113 of EstimationI
% The radar returns the straight-line distance from a point on the ground
% to the airplane position
% mup - predicted state
% H - linearized measurement matrix
    H = [(mup(1))/(sqrt(mup(1)^2 + mup(3)^2)) 0 (mup(3))/(sqrt(mup(1)^2 + mup(3)^2))];
end