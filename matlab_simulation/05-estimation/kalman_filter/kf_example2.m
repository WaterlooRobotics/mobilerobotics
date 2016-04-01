% Kalman Filter Example 2: 2D Omnidirectional AUV example
% 'Estimation 1' topic slide 77
% State : position and velocity in north, east directions
% Motion : motion in north and east directions
% Measurement : measure position in north and east directions
% Controller : Thrust in north and east directions

clear all
clc
close all

% Example number
example = 2;

% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('example2kf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.1;

% Get motion model and measurement model
[A,B,R,numStates] = motion_model(example,dt);
[C,D,Q,m] = measurement_model(example);

% Prior
mu = zeros(numStates,1); % mean (mu)
S = 1*eye(numStates);% covariance (Sigma)
% S = 0.1*eye(numStates);% covariance (Sigma)
% S = 0.01*eye(numStates);% covariance (Sigma)

[QRE, QRe] = eig(Q.QR);
[RE, Re] = eig (R);

% Store in a structure (State Space Model [ssm])
ssm.A = A;
ssm.B = B;
ssm.C = C;
ssm.D = D;
ssm.R = R;
ssm.Q = Q;
ssm.n = numStates;
ssm.m = m;

% Simulation Initializations
Tf = 10;
T = 0:dt:Tf;
u = 10*[sin(2*T);cos(T)];
numStates = length(A(1,:));
x = zeros(numStates,length(T));
x(:,1) = zeros(numStates,1);
m = length(C(:,1));
y = zeros(m,length(T));
mup_S = zeros(numStates,length(T));
mu_S = zeros(numStates,length(T));

% Frequency variable used for multirate kalman filter(eg 3). 
% Not used in eg 1 & 2
freq = 0;

%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(numStates,1);
    % Update state
    x(:,t) = A*x(:,t-1)+ B*u(:,t) + e;

    % Take measurement
    % Select a motion disturbance
    d = QRE*sqrt(QRe)*randn(m,1);
    % Determine measurement
    y(:,t) = C*x(:,t) + d;

    
    %% Kalman Filter Estimation
    [mu,S,mup,Sp,K] = kalman_filter(ssm,mu,S,u(:,t),y(:,t),example,t,freq);
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = [K(:,1); K(:,2)];
  
    %% Plot results
    figure(1);clf; hold on;
    plot(x(3,2:t),x(1,2:t), 'ro--')
    plot(y(2,2:t),y(1,2:t), 'gx')
    %plot(mup_S(3,1:t),mup_S(1,1:t), 'mx--')
    plot(mu_S(3,2:t),mu_S(1,2:t), 'bx--')
    ylabel('World X location [m]')
    xlabel('World Y location [m]')
    mu_pos = [mu(3) mu(1)];
    S_pos = [S(3,3) S(3,1); S(1,3) S(1,1)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and beliefs')
    legend('State', 'Measurement','Estimate')
    axis equal
    axis([-10 10 -5 15])
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
end

if (makemovie) close(vidObj); end

figure(2);clf;
plot(T,K_S');
title('Kalman gains as a function of time')
ylabel('Kalman Gain')
xlabel('Time')