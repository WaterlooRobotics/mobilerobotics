function features = searchForFeatures( X, map, rMax, thetaMax, measurementNoiseFactor )
    %% searchForFeatures( X, map, rMax, thetaMax, measurementNoiseFactor ) 
    % given a position on a map and a list of features (map), return
    % the index, and measurements to any features that are within range
    %

    features = [];

    countFeatures = size(map,2);
    for f = 1:countFeatures 
        rf = sqrt( sum( (X(1:2) - map(:,f)).^2 ) );
        bf = bearing( map(1,f), map(2,f), X(1), X(2), X(3) );

        if (rf <= rMax) && (thetaMax >= abs(bf)) 
            % feature is in view -- calculate a noisy measurement
            fm = [ rf; bf ] + measurementNoiseFactor * randn(2, 1);
            features = [features [fm; f]];
        end
    end
end


