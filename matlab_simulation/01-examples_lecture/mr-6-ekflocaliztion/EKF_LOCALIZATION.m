%{
EXTENDED KALMAN FILTER LOCALIZATION

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
    -Change map generation to random (can add instabilities)

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
u = [1 ; 0.3];

% Motion model
% Motion models are imported from seperate functions                         
R = [1e-4 0    0; 
     0    1e-4 0; 
     0    0    1e-5];
[eigenvectorR, eigenvalueR] = eig(R);

% Measurement type and noise
MEASUREMENT_TYPE = 3; % 1 - range, 2 - bearing, 3 - both
switch(MEASUREMENT_TYPE)
    case 1
        Q = 0.005;
    case 2
        Q = 0.005;
    case 3
        Q = [0.05 0 ; 
             0    0.05];
end
[eigenvectorQ, eigenvalueQ] = eig(Q);

% Sensor footprint
RANGE_MAX = 10 ; % meters
THETA_MAX = pi / 4; % rads (Note: Half the total field of view)

% Feature Map
randMapFlag=0;% Set=1:to randomize Map. 0:to load static map
if(randMapFlag==1)
    % Set the number of random features
    nFeatures = 10;
    featureMap = 10*rand(nFeatures,2);
    featureMap(:,1) = featureMap(:,1)-5; 
    featureMap(:,2) = featureMap(:,2)-2; 
else
    % Position of features on map [X Position, Y Position ; ...]
    featureMap = [ 5 5 ; 3  1 ; -4  5 ; -2  3 ; 0  4 ];
    nFeatures = length(featureMap(:,1));
end

% Simulation Initializations
nStates = length(x0);   % Number of states in model
x = zeros(nStates,length(T));   % Time history of states
x(:,1) = x0;    % Assigning intial conditions to state array
nMeasurements = length(Q(:,1)); % Number of measurements in model
y = zeros(nMeasurements,length(T)); % Time history of measurements
mup_S = zeros(nStates,length(T));   % Predicted states of Kalman filter
mu_S = zeros(nStates,length(T));    % Updated prediction of kalman filter states
selectedFeature = zeros(2,1);   % Array holds coordinates of considered feature

%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    motionDisturbance = eigenvectorR*sqrt(eigenvalueR)*randn(nStates,1);
    % Update state with prediction
    x(:,t) = simpleRobotMotionModel(x(:,t-1),u) + motionDisturbance;
    % Take measurement
    [tempMeasurement,featureInViewFlag] = get2dpointmeasurement(featureMap,x(:,t),RANGE_MAX,THETA_MAX,Q,MEASUREMENT_TYPE);
    y(1:length(tempMeasurement),t)=tempMeasurement;
    
    %% Extended Kalman Filter
    % Call EKF function
    % Find first feature in view
        %-Currently, EKF not programmed to use multiple measurments if they are available
        %-further away objects seems to improve bearning uncertainty, but whether or
        %   not they are used depend on where they are in the featuresMap.
        %-Can change find argument if want to consider additional measurements
    index = find(featureInViewFlag==1,1);
    % Determine coordinates of feature used for localization
    selectedFeature = featureMap(index,:);
    % Localize with respect to chosen feature
    [ mu, S, mup, K, measurementError ] = multisensorekf(selectedFeature, MEASUREMENT_TYPE, ...
        index, mu,u, S, y(:,t), Q, R );

    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    Innovation(:,t) = measurementError;
    %% Plot results
    %Plotting initializations
    figure(1);clf; hold on;
    axis equal
    axis([-5 5 -2 8]) % Set to maximum of robot trajectory and map features
    
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

%Plot measurement vs prediction error time series
figure(2);clf; hold on;
plot(T,Innovation)
title('Error Between Measurement and Prediction')
xlabel('Time [s]')
ylabel('Innovation')
switch (MEASUREMENT_TYPE)
    case 1
        legend('Range Error')
    case 2
        legend('Bearing Error')
    case 3
        legend('Range Error','Bearing Error')
end

if (makemovie) close(vidObj); end