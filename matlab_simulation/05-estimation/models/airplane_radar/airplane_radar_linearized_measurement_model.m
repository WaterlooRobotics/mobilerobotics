function M = airplane_radar_linearized_measurement_model(mup, ~)
    M = [(mup(1))/(sqrt(mup(1)^2 + mup(3)^2)) 0 (mup(3))/(sqrt(mup(1)^2 + mup(3)^2))];
end