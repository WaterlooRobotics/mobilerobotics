% Extended Kalman filter example
%
% Radar measurement of airplane position while flying at a constant
% altitude and velocity. In this case the motion model is linear, but the
% measurement model is nonlinear.
%
% This example can be found on slide 111 of Estimation-I in the notes.
clear;clc;

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.1;

% Initial State [horizontal distance, horizontal velocity, height]
x0 = [20 -2 3]';

% Prior
mu = [22 -1.8 6.5]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)

% Discrete motion model
Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];

% Disturbance model
R = [.0001 0 0; 0 .0001 0 ; 0 0 .0001];
[RE, Re] = eig (R);

% Measurement model defined below
Q = [.0001];

% Simulation Initializations
Tf = 10;
T = 0:dt:Tf;
n = length(Ad(1,:));
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));

% Variables to store during iteration
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
K_S = zeros(n,n,length(T));

%% Main loop
for t=2:length(T)
    %%% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = Ad*x(:,t-1) + e;

    % Take measurement
    % Select a motion disturbance
    d = sqrt(Q)*randn(m,1);
    % Determine measurement
    y(:,t) = sqrt(x(1,t)^2 + x(3,t)^2) + d;


    %%% Extended Kalman Filter Estimation
    [mu, S, mup, K] = ekf(mu, S, y(:,t), ...
                          @airplane_radar_motion_model, ...
                          @airplane_radar_measurement_model, ...
                          @airplane_radar_linearized_motion_model, ...
                          @airplane_radar_linearized_measurement_model, ...
                          Q, R);

    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = K;


    %%% Plot results
    figure(1);clf; hold on;
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot([20 -1],[0 0],'b--')
    plot(x(1,2:t),x(3,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(3,2:t), 'bx--')
    mu_pos = [mu(1) mu(3)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);

    title('True state and belief')

    axis equal
    axis([-1 20 -10 10])
    if (makemovie) writeVideo(vidObj, getframe(gca)); end

end
xlabel('ground distance');
ylabel('height');
legend('True State', 'Belief');
if (makemovie) close(vidObj); end

