%{
Extended Kalman Filter Localization
Author: Rhyse
Date: March 4, 2016
VERSION HISTORY
V1.0 - Provided by prof Waslander
V1.1 - Improved readability
        - Spacing, comments, plot labels
        - Constant and variables more meaningful and agree with citation style
     - Added circle() function for plotting functionality
     - Removed predicted mean from figure(2) due to unseen benefit
     - Added instructions
V1.2 - Feature measurement moved to separate function
     - Removed old EKF_Localiztion

INTRUCTIONS
You can observe the effects of:
-Bad prior
    -Change mu with respect to x0
-Motion model disturbance
    -Increase or decrease values in [R]
-Measurement model disturbance
    -Increase or decrease values in [Q]
-Measurement types
    -Change MEASUREMENT_TYPE
-Sensor performance
    -Change RANGE_MAX or THETA_MAX
-Feature density
    -Add/remove features from featureMap

EXPLAIN WHY ONLY USE ONE MEASUREMENT 
-EKF not programmed to use multiple measurments if they are available
-further away objects seems to improve bearning uncertainty, but whether or
not they are used depend on where they are in the featuresMap. Consider
using randperm

To do
-Creative example that show concepts
-Feature selection methods into environment function
-Test
%}
clear;clc;

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ekflocalization.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

%% Initialization

% Time
Tf = 20;
dt = 0.1;
T = 0:dt:Tf;

% Initial State
x0 = [0 0 0]'; % [X position, Y position, Angle(rad)]'

% Prior
mu = [0 0 0]'; % mean (mu) [X position, Y position, Angle(rad)]'
S = 0.1 * eye(3);% covariance (Sigma)

% Control inputs
u = ones(2, length(T));
u(2,:)=0.3 * u(2,:);

% Motion Disturbance model
motion_model = @ (x,u) [x(1) + u(1)*cos(x(3))*dt;
                        x(2) + u(1)*sin(x(3))*dt;
                        x(3) + u(2)*dt]
linearized_motion_model = @ (mu,u)[ 1 0 -u(1)*sin(mu(3))*dt;
                                    0 1 u(1)*cos(mu(3))*dt;
                                    0 0 1];
R = [1e-4 0 0; 
     0 1e-4 0; 
     0 0 1e-5];
[eigenvectorR, eigenvalueR] = eig(R);

% Measurement type and noise
MEASUREMENT_TYPE = 1; % 1 - range, 2 - bearing, 3 - both
switch(MEASUREMENT_TYPE)
    case 1
        Q = 0.005;
    case 2
        Q = 0.005;
    case 3
        Q = [0.05 0;
              0 0.05];
end
[eigenvectorQ, eigenvalueQ] = eig(Q);

% Sensor footprint
RANGE_MAX = 10 ; % meters
THETA_MAX = pi / 4; % rads

% Feature Map
% Position of features on map [X Position, Y Position ; ...]
featureMap = [ 5 5;  
               3  1;
               -4  5;
               -2  3;
               0  4];
nFeatures = length(featureMap(:,1));

% Simulation Initializations
nStates = length(x0);
x = zeros(nStates,length(T));   % Time history of states
x(:,1) = x0;
nMeasurements = length(Q(:,1));
y = zeros(nMeasurements,length(T)); % Time history of measurements
mup_S = zeros(nStates,length(T));
mu_S = zeros(nStates,length(T));
selectedFeature = zeros(2,1);
featureFoundFlag=0; %Switches to 1 when feature is found. Used to exit for loop

%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    motionDisturbance = eigenvectorR*sqrt(eigenvalueR)*randn(nStates,1);
    % Update state with prediction
    x(:,t) = motion_model(x(:,t-1),u(:,t)) + motionDisturbance;
    % Take measurement
    [p,featureInViewFlag] = get2dpointmeasurement(featureMap,x(:,t),RANGE_MAX,THETA_MAX,Q,MEASUREMENT_TYPE);
    y(1:length(p),t)=p;
    
    %% Extended Kalman Filter
    % Predicted mean. Needed to calculated linearized measurement model
    mup =    [mu(1) + u(1,t)*cos(mu(3))*dt;
              mu(2) + u(1,t)*sin(mu(3))*dt;
              mu(3) + u(2,t)*dt];
    % Store results
    mup_S(:,t) = mup;
    
    % Call EKF function
    for i=1:nFeatures
        
        % If a feature is in view and no features have been observed yet
        if (featureInViewFlag(i)&~featureFoundFlag)
            featureFoundFlag=1; %Robot has seen a feature for time t
            selectedFeature = featureMap(i,:);
            featureRange = sqrt((selectedFeature(1)-mup(1))^2 + ...
                (selectedFeature(2)-mup(2))^2);
            % Depending on which MEASUREMENT_TYPE is selected
                % Define the measurement models as fn(mup)
                % Do the EKF algoithm to get updated states
                % Calculate error between the measured and predicted [I]
            switch(MEASUREMENT_TYPE) 
                case 1
                    measurement_model = @ (mup) ...
                        sqrt((selectedFeature(1)-mup(1))^2 + ...
                        (selectedFeature(2)-mup(2))^2);
                    linearized_measurement_model = @ (mup) ...
                        [ -(selectedFeature(1)-mup(1))/featureRange ...
                          -(selectedFeature(2)-mup(2))/featureRange ...
                          0];
                    [mu, S, mup, K] = ekf(mu, u(:,t), S, y(i,t), motion_model, ...
                        measurement_model, linearized_motion_model, ...
                        linearized_measurement_model,Q,R);
                    I = y(i,t) - measurement_model(mup);
                    Inn(t) = I;
                case 2
                    measurement_model = @ (mup) ...
                        (atan2(selectedFeature(2)-mup(2),...
                        selectedFeature(1)-mup(1)) - mup(3));
                    linearized_measurement_model = @ (mup) ...
                        [ (selectedFeature(2)-mup(2))/featureRange^2 ...
                          -(selectedFeature(1)-mup(1))/featureRange^2 ...
                          -1];
                    [mu, S, mup, K] = ekf(mu, u(:,t), S, y(i,t), motion_model, ...
                        measurement_model, linearized_motion_model, ...
                        linearized_measurement_model,Q,R);
                    I = y(i,t) - measurement_model(mup);
                    I = mod(I+pi,2*pi)-pi;
                    Inn(t) = I;
            Inn(t) = I;
                case 3
                    measurement_model = @ (mup) ...
                        [sqrt((selectedFeature(1)-mup(1))^2 + (selectedFeature(2)-mup(2))^2);
                        (atan2(selectedFeature(2)-mup(2),selectedFeature(1)-mup(1)) - mup(3))];
                    linearized_measurement_model = @ (mup) ...
                        [ -(selectedFeature(1)-mup(1))/featureRange ...
                        -(selectedFeature(2)-mup(2))/featureRange ...
                        0;
                        (selectedFeature(2)-mup(2))/featureRange^2 ...
                        -(selectedFeature(1)-mup(1))/featureRange^2 ...
                        -1];
                    [mu, S, mup, K] = ekf(mu, u(:,t), S, y(2*(i-1)+1:2*i,t),...
                        motion_model,measurement_model, ...
                        linearized_motion_model, linearized_measurement_model,Q,R);
                    I = y(2*(i-1)+1:2*i,t)-measurement_model(mup);
                    I(2) = mod(I(2)+pi,2*pi)-pi;
                    Inn(:,t) = I;
            end
            % If ____ go to debug mode
                % Case 1: Radius error > 10
                % Case 2: Bearing error > 10 rad
                % Case 3: Norm of radius and bearing error > 10
            if (norm(I) > 10)
                keyboard;
            end
        end
    end
    %
    % Reset feature found flag
        featureFoundFlag=0;
    % If no features in view no updated prediction
    if (sum(featureInViewFlag)==0) mu = mup; end
    
    % Store results
    mu_S(:,t) = mu;
    %% Plot results
    %Plotting initializations
    figure(1);clf; hold on;
    axis equal
    axis([-4 6 -1 7]) % Set to maximum of robot trajectory and map features
    
    % PLOT
    % Plot states
    plot(x(1,1:t),x(2,1:t), 'ro--')
    % Plot map features
    plot(featureMap(:,1),featureMap(:,2),'go', 'MarkerSize',10,'LineWidth',2);
    
    
    % Plot initial prediction (model propgation)
    plot(mup_S(1,1:t),mup_S(2,1:t), 'go--')
    % Plot updated prediction
    plot(mu_S(1,1:t-1),mu_S(2,1:t-1), 'bx--')
    
    %Plot error ellipses
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.95);
    error_ellipse(S_pos,mu_pos,0.999);
    % Plot robot measurement to inview features
    for i = 1:nFeatures
        if (featureInViewFlag(i))
            plot(featureMap(i,1),featureMap(i,2),'mx', 'MarkerSize',10,'LineWidth',2)
            if (MEASUREMENT_TYPE==1) 
                circle(1,x(1:2,t), y(i,t)); 
            end
            if (MEASUREMENT_TYPE==2) 
                plot([x(1,t) x(1,t)+10*cos(y(i,t)+x(3,t))],...
                    [ x(2,t) x(2,t)+10*sin(y(i,t)+x(3,t))], 'c');
            end
            if (MEASUREMENT_TYPE==3) 
                plot([x(1,t) x(1,t)+y(2*(i-1)+1,t)*cos(y(2,t)+x(3,t))],...
                    [ x(2,t) x(2,t)+y(2*(i-1)+1,t)*sin(y(2*i,t)+x(3,t))], 'c');
            end
        end
    end
    drawnow;
    
    if (makemovie) writeVideo(vidObj, getframe(gca)); end

end
% Add plotting options for figure 1
title('Range & Bearing Measurements for Localization')
xlabel('X Postion [Unit]')
ylabel('Y Position [Unit]')
legend('Robot State','Map Feature','Initial State Prediction',...
    'Updated State Prediction')

if (makemovie) close(vidObj); end

%Plot measurement vs prediction error time series
figure(2);clf; hold on;
plot(T,Inn)
title('Error Between Measurement and Prediction')
xlabel('Time [s]')
ylabel('Error')
switch (MEASUREMENT_TYPE)
    case 1
        legend('Range Error')
    case 2
        legend('Bearing Error')
    case 3
        legend('Range Error','Bearing Error')
end