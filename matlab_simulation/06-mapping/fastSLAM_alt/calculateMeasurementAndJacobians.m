function [y, Hm, Hx] = calculateMeasurementAndJacobians( f_mu, p_mup )
    %% calculateMeasurementJacobians( f_mu, p_mup )
    % Using the predicted particle position and the previous belief
    % of the feature, calculate the Jacobians with respect to X, and M.
    dx = f_mu(1) - p_mup(1);
    dy = f_mu(2) - p_mup(2);
    r_sq = dx*dx + dy*dy;
    r = sqrt(r_sq);

    b = bearing( f_mu(1), f_mu(2), p_mup(1), p_mup(2), p_mup(3) );
    
    y = [ r; b ];

    Hm = [  dx/r,      dy/r;
           -dy/r_sq,  dx/r_sq ];
       
    Hx = [ -dx/r,     -dy/r,      0;
            dy/r_sq, -dx/r_sq, -1];
end
