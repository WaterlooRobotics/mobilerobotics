function [ mu, S, mup, K, I ] = multisensorekf(selectedFeature, MEASUREMENT_TYPE,index, mu,u, S, y, motion_model, linearized_motion_model, Q, R )
%UNTITLED6 The additional setup required for robots with variable sensor
%capabilities

%Sensor types listed as MEASUREMENT_TYPE
% Inputs:   selectedFeature: Location of feature using for localization [X pos, Y pos]
%           MEASUREMENT_TYPE: 1 - range, 2 - bearing, 3 - both
%           index: Which feature is being considered
%           mu: State estimate at time t-1
%           u: Input at time t
%           S: Covariance matrix
%           y: Measurement at time t
%           motion_model: function handle of non-linear motion model
%           linearized_motion_model: motion_model linearized
%           Q: Measurement disturbance matrix
%           R: Motion disturbance matrix
% Outputs:  mu: Updated state prediction
%           S: Updated covariance matrix
%           mup: State prediction using motion model only
%           K: Kalman gain
%           I: Error between measurement and model prediction

%% Initialize variables
K=0;
I=0;
S=S;

mup = motion_model(mu,u);

%If there exists a feature in view
if (selectedFeature)
        featureRange = sqrt((selectedFeature(1)-mup(1))^2 + ...
            (selectedFeature(2)-mup(2))^2);
        % Depending on which MEASUREMENT_TYPE is selected
            % Define the measurement models as fn(mup)
            % Do the EKF algoithm to get updated states
            % Calculate error between the measured and predicted [I]
        switch(MEASUREMENT_TYPE) 
            case 1
                measurement_model = @ (mup,noInput) ...
                    sqrt((selectedFeature(1)-mup(1))^2 + ...
                    (selectedFeature(2)-mup(2))^2);
                linearized_measurement_model = @ (mup,noInput) ...
                    [ -(selectedFeature(1)-mup(1))/featureRange ...
                      -(selectedFeature(2)-mup(2))/featureRange ...
                      0];
                [mu, S, mup, K] = ekf(mu, S, y(index), motion_model, ...
                    measurement_model, linearized_motion_model, ...
                    linearized_measurement_model, Q, R, u);
                I = y(index) - measurement_model(mup);
            case 2
                measurement_model = @ (mup,noInput) ...
                    (atan2(selectedFeature(2)-mup(2),...
                    selectedFeature(1)-mup(1)) - mup(3));
                linearized_measurement_model = @ (mup,noInput) ...
                    [ (selectedFeature(2)-mup(2))/featureRange^2 ...
                      -(selectedFeature(1)-mup(1))/featureRange^2 ...
                      -1];
                [mu, S, mup, K] = ekf(mu, S, y(index), motion_model, ...
                    measurement_model, linearized_motion_model, ...
                    linearized_measurement_model, Q, R, u);
                I = y(index) - measurement_model(mup);
                I = mod(I+pi,2*pi)-pi;
            case 3
                measurement_model = @ (mup,noInput) ...
                    [sqrt((selectedFeature(1)-mup(1))^2 + (selectedFeature(2)-mup(2))^2);
                    (atan2(selectedFeature(2)-mup(2),selectedFeature(1)-mup(1)) - mup(3))];
                linearized_measurement_model = @ (mup,noInput) ...
                    [ -(selectedFeature(1)-mup(1))/featureRange ...
                    -(selectedFeature(2)-mup(2))/featureRange ...
                    0;
                    (selectedFeature(2)-mup(2))/featureRange^2 ...
                    -(selectedFeature(1)-mup(1))/featureRange^2 ...
                    -1];
                [mu, S, mup, K] = ekf(mu, S, y(2*(index-1)+1:2*index),...
                    motion_model,measurement_model, ...
                    linearized_motion_model, linearized_measurement_model,Q,R, u);
                I = y(2*(index-1)+1:2*index)-measurement_model(mup);
                I(2) = mod(I(2)+pi,2*pi)-pi;
        end
        % If ____ go to debug mode
            % Case 1: Radius error > 10
            % Case 2: Bearing error > 10 rad
            % Case 3: Norm of radius and bearing error > 10
        if (norm(I) > 10) keyboard; end
else
    % If no features in view no updated prediction.
    mu = mup;
end
end

