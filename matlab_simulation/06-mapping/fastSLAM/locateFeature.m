function x = locateFeature( p_mu, f_y )
    %% locateFeature( p_mu, f_y )
    % Calculate an (x,y) coordinate for a feature given a bearing and range measurement
    r = f_y(1);
    b = f_y(2);
    
    % Note that we have to add angle of the robot body here as we are
    % moving from the World frame to the Robot frame.
    x = [ p_mu(1) + r * cos(b+p_mu(3)); 
          p_mu(2) + r * sin(b+p_mu(3));];
end
