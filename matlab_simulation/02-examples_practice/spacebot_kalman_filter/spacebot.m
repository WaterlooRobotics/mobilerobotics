%% Spacebot
% Includes a KF and PD control
clear; clc;

%% Simulation setup
% Time
t0 = 0;
tf = 10;
dt = 0.01;
T = t0:dt:tf;
M = 20;

% Initial position
x0 = [2 0 2  0 2 0]';

% Desired position
xd = [10 0 4 0 5 0]';

%% Motion model

% State x = [x vx y vy z vz]
A=[1 dt 0 0 0 0; 0 1 0 0 0 0;
   0 0 1 dt 0 0; 0 0 0 1 0 0;
   0 0 0 0 1 dt; 0 0 0 0 0 1];

B=[0 0 0; dt/M 0 0; 0 0 0; 0 dt/M 0; 0 0 0; 0 0 dt/M];
n = length(A);
m = length(B(1,:));

%% Measurement model
C=[1 0 0 0 0 0; -1 0 0 0 0 0; 
   0 0 1 0 0 0; 0 0 -1 0 0 0;
   0 0 0 0 1 0; 0 0 0 0 -1 0];
q = length(C(1,:));
y_off = [ .50; -19.50; .50; -9.50; .50; -9.50];



%% PD control gains
Kp = 100;
Kd = 40;

%PD gain matrix
Kc = [ Kp Kd 0 0 0 0;
       0 0 Kp Kd 0 0;
       0 0 0 0 Kp Kd];


%% Kalman covariances
Re = 0.0001*eye(q);
Qe = 0.0001*eye(n);
[QeV, Qev] = eig(Qe);
[ReV, Rev] = eig(Re);

%% Simulation

% Setup storage variables
x = zeros(n,length(T));
x(:,1) = x0;
u = zeros(m,length(T)-1);
y = zeros(q,length(T));

mu = x0;
S = 2*eye(n);
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
S_S = zeros(n,n,length(T));
Ke_S = zeros(q,n,length(T));

%% Main loop
for t=2:length(T)-1
    %% PD control with desired state offset
    u(:,t)= Kc*(xd-mu);
    
    %% Simulation of motion and measurements
    % Select a motion disturbance
    e = ReV*sqrt(Rev)*randn(n,1);
    % Update state
    x(:,t) = A*x(:,t-1)+ B*u(:,t) + e;

    % Take measurement
    % Select a motion disturbance
    d = QeV*sqrt(Qev)*randn(n,1);
    % Determine measurement
    y_raw(:,t) = C*x(:,t) + y_off  + d;

    %% Kalman Filter Estimation
    % Prediction update
    mup = A*mu + B*u(:,t);
    Sp = A*S*A' + Re;

    % Modify measurements to use in KF
    y(:,t) = y_raw(:,t) - y_off;

    % Measurement update
    Ke = Sp*C'*inv(C*Sp*C'+Qe);
    mu = mup + Ke*(y(:,t)-C*mup);
    S = (eye(n)-Ke*C)*Sp;
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    S_S(:,:,t) = S;
    Ke_S(:,:,t) = Ke;
    
end

%% Plot results
figure(1); clf; hold on;
plot(x(1,2:t),x(3,2:t), 'ro--')
plot(mu_S(1,2:t),mu_S(3,2:t), 'bx--')
for i=2:round(1/dt):t
    mu_pos = [mu_S(1,i) mu_S(3,i)];
    S_pos = [S_S(1,1,i) S_S(1,3,i); S_S(3,1,i) S_S(3,3,i)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
end
title('True state and beliefs')
legend('State', 'Estimate')

figure(2); clf;
Ke_S2 = reshape(Ke_S, q*n,1,length(T));
plot(T,squeeze(Ke_S2)');
title('Kalman Gain')

figure(3); clf; hold on;
subplot(2,1,1);
plot(T(1:end-1), u);
title('Control Inputs');
subplot(2,1,2);
plot(T(1:end-1), xd*ones(1,length(T)-1)-x(:,1:end-1), '--')
title('State Errors')

figure(4); clf; hold on;
subplot(3,1,1)
plot(T(2:t),x(1,2:t), 'ro--')
subplot(3,1,2)
plot(T(2:t),x(3,2:t), 'ro--')
subplot(3,1,3)
plot(T(2:t),x(5,2:t), 'ro--')
title('Position trajectories')
