function [ measurement,featureInViewFlag ] = get2dpointmeasurement( featureMap,state,RANGE_MAX,THETA_MAX,Q,MEASUREMENT_TYPE )
%get2dpointmeasurement Generate range&|bearing measurement to point object
% Uses a robot with bearing and range measurement to take a
% reading with included disturbances.
% Inputs:   featureMap: 2D array holding [x position, y position ; ...]
%           state: True robot state at considered time step [x position, y position, theta]
%           RANGE_MAX: Maximum range of scanner
%           THETA_MAX: Maximum bearing of scanner [rad]
%           MEASUREMENT_TYPE: 1 - range, 2 - bearing, 3 - both
% Outputs:  measurement: Vector containing range&|bearing measurement for
%                           all objects in view
%           featureInViewFlag: A vector which indicates which features are
%                               in view

%% Initializations
% Calculating the number of features
nFeatures = length(featureMap(:,1));

% Initialize inview vector
featureInViewFlag = zeros(nFeatures,1); 

% Measurement disturbance model initialization
[eigenvectorQ, eigenvalueQ] = eig(Q);

% Calculating number of measurement
nMeasurements = length(Q(:,1));

if (MEASUREMENT_TYPE==1 | MEASUREMENT_TYPE==2)
    measurement = zeros(nFeatures,1);
elseif (MEASUREMENT_TYPE==3)
    measurement = zeros(2*nFeatures,1);
else
    'Error: Invalid Measurement Type. Choose 1,2 or 3'
    return
end

%% Main Calculations
for i=1:nFeatures
        % If feature is visible
        if (inview(featureMap(i,:) , state , RANGE_MAX , THETA_MAX))
            featureInViewFlag(i) = 1;   % Set feature to visible
            selectedFeature = featureMap(i,:);
            % Select a motion disturbance
            measurementDisturbance = eigenvectorQ * sqrt(eigenvalueQ) * randn(nMeasurements,1);
            % Determine measurement
            switch(MEASUREMENT_TYPE)
                case 1
                    measurement(i,1) = max(0.001,sqrt((selectedFeature(1)-state(1))^2 +...
                        (selectedFeature(2)-state(2))^2) +...
                        measurementDisturbance);
                case 2
                    measurement(i,1) = [atan2(selectedFeature(2)-state(2),...
                        selectedFeature(1)-state(1))-state(3)] +...
                        measurementDisturbance;
                case 3
                    measurement(2*(i-1)+1:2*i,1) = ...
                        [max(0.001, sqrt((selectedFeature(1)-state(1))^2 + (selectedFeature(2)-state(2))^2));
                        atan2(selectedFeature(2)-state(2),selectedFeature(1)-state(1))-state(3)] + measurementDisturbance;
            end
        end
end
end

