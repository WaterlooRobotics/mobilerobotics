function M = airplane_radar_measurement_model(mup)
    M = sqrt(mup(1)^2 + mup(3)^2);
end