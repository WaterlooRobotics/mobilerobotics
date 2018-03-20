function [y, H] = calculateFeatureMeasurement( f_mu, p_mup )
    %% calculateFeatureMeasurement( f_mu, r_mu )
    % Using the predicted particle position and the previous belief
    % of the feature, calculate what the measurement should be and 
    % the linearized measurement model, H.
    dx = f_mu(1) - p_mup(1);
    dy = f_mu(2) - p_mup(2);
    r_sq = dx*dx + dy*dy;
    r = sqrt(r_sq);

    b = bearing( f_mu(1), f_mu(2), p_mup(1), p_mup(2), p_mup(3) );

    y = [ r; b ];
    H = [  dx/r,     dy/r;
          -dy/r_sq,  dx/r_sq ];
end

