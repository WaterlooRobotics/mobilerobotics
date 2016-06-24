% Extended Kalman filter example
clear;clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.1;
v = 5.5;
w=-0.9;

% Initial State
x0 = [0 0 0]';

% Prior
mu = [0 0 0]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)

% Measurement model defined below
R=diag([0.0005 0.0005 0.5*(pi/180)^2]); %degree to rad conversion
[RE, Re] = eig (R);
Q=diag([0.005 0.005 10*(pi/180)^2]);

% Simulation Initializations
Tf = 10;
T = 0:dt:Tf;
n = length(mu);
x = zeros(n,length(T));
x_ideal = zeros(n,length(T));
x(:,1) = x0;
x_ideal(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x_ideal(:,t) = omnibot_motion_model(x_ideal(:,t-1),w,v,dt);
    x(:,t) = omnibot_motion_model(x(:,t-1),w,v,dt) + e;

    % Take measurement
    % Select a motion disturbance
    d = sqrt(Q)*randn(m,1);
    % Determine measurement
    y(:,t) = omnibot_sensor_model(x_ideal(:,t)) + d;
    Y=y(:,t);


    %% Extended Kalman Filter Estimation
    [mu,S,K,mup] = EKF(mu,S,w,v,dt,Y,@omnibot_motion_model,@omnibot_sensor_model,...
        @omnibot_linearize_motion_model,@omnibot_linearize_sensor_model,R,Q);

    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    %K_S(:,t) = K;


    %% Plot results
    figure(1);clf; hold on;
    %plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    %plot([20 -1],[0 0],'b--')
    plot(x(1,2:t),x(2,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and belief')
    axis equal
    axis([-8 8 -14 2])
    legend('True Path', 'EKF path');
    if (makemovie) writeVideo(vidObj, getframe(gca)); end

end

if (makemovie) close(vidObj); end