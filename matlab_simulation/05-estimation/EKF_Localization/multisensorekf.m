function [ mu, S, mup, K, measurementError ] = multisensorekf(selectedFeature, MEASUREMENT_TYPE,index, mu,u, S, y, Q, R )
%The additional setup required for robots with variable sensor
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
measurementError=0;

mup = simpleRobotMotionModel(mu,u);

%If there exists a feature in view
if (selectedFeature)
    % Calculate Feature range
    featureRange = sqrt((selectedFeature(1)-mup(1))^2 + ...
        (selectedFeature(2)-mup(2))^2);
    % Get measurement Model
    measurement_model = ...
        simpleRobotMeasurementModel(selectedFeature,MEASUREMENT_TYPE);
    % Get linearized motion model
    linearized_measurement_model = ...
        simpleLinearizedRobotMeasurementModel( selectedFeature, featureRange, MEASUREMENT_TYPE );
    % Depending on which MEASUREMENT_TYPE is selected
        % Do the EKF algoithm to get updated states
        % Calculate error between the measured and predicted [I]
    switch(MEASUREMENT_TYPE) 
        case 1
            [mu, S, mup, K] = ekf(mu, S, y(index), @simpleRobotMotionModel, ...
                measurement_model, @simpleLinearizedRobotMotionModel, ...
                linearized_measurement_model, Q, R, u);
            measurementError = y(index) - measurement_model(mup);
        case 2
            [mu, S, mup, K] = ekf(mu, S, y(index), @simpleRobotMotionModel, ...
                measurement_model, @simpleLinearizedRobotMotionModel, ...
                linearized_measurement_model, Q, R, u);
            measurementError = y(index) - measurement_model(mup);
            measurementError = mod(measurementError+pi,2*pi)-pi;
        case 3
            [mu, S, mup, K] = ekf(mu, S, y(2*(index-1)+1:2*index),...
                @simpleRobotMotionModel,measurement_model, ...
                @simpleLinearizedRobotMotionModel, linearized_measurement_model,Q,R, u);
            measurementError = y(2*(index-1)+1:2*index)-measurement_model(mup);
            measurementError(2) = mod(measurementError(2)+pi,2*pi)-pi;
    end
    % If ____ go to debug mode
        % Case 1: Radius error > 10
        % Case 2: Bearing error > 10 rad
        % Case 3: Norm of radius and bearing error > 10
    if (norm(measurementError) > 10) keyboard; end
else
    % If no features in view no updated prediction.
    mu = mup;
end
end

