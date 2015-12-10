% Kalman filter example
clear;clc;

% Discrete time step
dt = 0.1;

% Prior
mu = zeros(4,1); % mean (mu)
S = 1*eye(4);% covariance (Sigma)
%S = 0.1*eye(4);% covariance (Sigma)
%S = 0.01*eye(4);% covariance (Sigma)

% Continuous motion model (2-D)
b = 1;
m = 2;
A = [ 0 1 0 0; 0 -b/m 0 0; 0 0 0 1; 0 0 0 -b/m];
B = [0 0 ;1/m 0; 0 0; 0 1/m];
R = [.01 0 0 0; 0 .01 0 0; 0 0 .01 0; 0 0 0 .01];
%R = [.001 0 0 0; 0 .001 0 0; 0 0 .001 0; 0 0 0 .001];
%R = [.0001 0 0 0; 0 .0001 0 0; 0 0 .0001 0; 0 0 0 .0001];
[RE, Re] = eig (R);

% Measurement model
C = zeros(2,4);
C(1,1) = 1;
C(2,3) = 1;
D = zeros(2,2);
QR = [.4 -0.1; -0.1 .1];
Q = [.04 -0.01; -0.01 .01];
%Q = [.004 -0.001; -0.001 .001];
%Q = [.0004 -0.0001; -0.0001 .0001];
[QRE, QRe] = eig (QR);
[QE, Qe] = eig (Q);

% Form continuous system
sysc=ss(A,B,C,D);

% zoh discretization
sysd = c2d(sysc,dt,'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

% Simulation Initializations
Tf = 10;
T = 0:dt:Tf;
u = 10*[sin(2*T);cos(T)];
n = length(Ad(1,:));
x = zeros(n,length(T));
x(:,1) = zeros(n,1);
m = length(Cd(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = Ad*x(:,t-1)+ Bd*u(:,t) + e;

    % Take measurement
    % Select a motion disturbance
    d = QRE*sqrt(QRe)*randn(m,1);
    % Determine measurement
    y(:,t) = Cd*x(:,t) + d;

    
    %% Kalman Filter Estimation
    % Prediction update
    mup = Ad*mu + Bd*u(:,t);
    Sp = Ad*S*Ad' + R;

    % Measurement update
    K = Sp*Cd'*inv(Cd*Sp*Cd'+Q);
    mu = mup + K*(y(:,t)-Cd*mup);
    S = (eye(n)-K*Cd)*Sp;
    
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
    mu_pos = [mu(3) mu(1)];
    S_pos = [S(3,3) S(3,1); S(1,3) S(1,1)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and beliefs')
    legend('State', 'Measurement','Estimate')
    axis equal
    axis([-10 10 -5 15])
end

figure(2);clf;
plot(T,K_S');
title('Kalman gains as a function of time')

figure(3);clf;
plot(T(2:t),x(:,2:t)-mu_S(:,2:t))

