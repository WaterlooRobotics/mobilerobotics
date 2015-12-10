% Unscented Kalman filter example
clear;clc;
reset(RandStream.getDefaultStream);

% Discrete time step
dt = 0.1;

% Initial State
x0 = [20 -2 3]';

% Prior
mu = [22 -1.8 6]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)
mu_u = mu;
S_u = S;

% Discrete motion model
Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];

R = [.001 0 0; 0 .01 0 ; 0 0 .001];
[RE, Re] = eig (R);

% Measurement model defined below
Q = [.001];

% Simulation Initializations
Tf = 10;
T = 0:dt:Tf;
n = length(Ad(1,:));
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
K_S = zeros(n,length(T));
mup_Su = zeros(n,length(T));
mu_Su = zeros(n,length(T));
K_Su = zeros(n,length(T));

% Unscented transform parameters
alpha = 1.2;
kappa = 0;
beta = 2;
lambda = alpha^2*(n+kappa)-n;
w_m(1) = lambda/(n+lambda);
w_c(1) = lambda/(n+lambda) + (1-alpha^2+beta);
w_m(2:2*n+1) = 1/(2*(n+lambda));
w_c(2:2*n+1) = 1/(2*(n+lambda));


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = Ad*x(:,t-1) + e;

    % Take measurement
    % Select a motion disturbance
    d = sqrt(Q)*randn(m,1);
    % Determine measurement
    y(:,t) = sqrt(x(1,t)^2 + x(3,t)^2) + d;


    %% Extended Kalman Filter Estimation
    % Prediction update
    mup = Ad*mu;
    Sp = Ad*S*Ad' + R;

    % Linearization
    Ht = [(mup(1))/(sqrt(mup(1)^2 + mup(3)^2)) 0 (mup(3))/(sqrt(mup(1)^2 + mup(3)^2))];

    % Measurement update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup + K*(y(:,t)-sqrt(mup(1)^2 + mup(3)^2));
    S = (eye(n)-K*Ht)*Sp;

    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = K;

    %% Unscented Kalman Filter Estimation
    % Prediction update
    nS_u = sqrtm((n+lambda)*S_u);
    xp_sp(:,1) = mu_u;
    x_sp(:,1) = Ad*xp_sp(:,1);
    for i=1:n
        % Sigma points prior to propagation
        xp_sp(:,i+1) = mu_u + nS_u(:,i);
        xp_sp(:,n+i+1) = mu_u - nS_u(:,i);
        % Sigma points after propagation
        x_sp(:,i+1) = Ad*xp_sp(:,i+1);
        x_sp(:,n+i+1) = Ad*xp_sp(:,n+i+1);
    end
    
    mup_u = zeros(n,1);
    for i=1:2*n+1
        mup_u = mup_u +w_m(i)*x_sp(:,i);
    end
    Sp_u = zeros(n);
    for i=1:2*n+1
        Sp_u = Sp_u + w_c(i)*((x_sp(:,i)-mup_u)*(x_sp(:,i)-mup_u)');
    end
    Sp_u = Sp_u + R;
      
    % Measurement update
    nSp_u = sqrtm((n+lambda)*Sp_u);
    xp_sm(:,1) = mup_u;
    y_sm(:,1) = sqrt(xp_sm(1,1)^2+xp_sm(3,1)^2);
    for i=1:n
        % Sigma points prior to measurement
        xp_sm(:,i+1) = mup_u + nSp_u(i,:)';
        xp_sm(:,n+i+1) = mup_u - nSp_u(i,:)';
        % Measurement model applied to sigma points
        y_sm(:,i+1) = sqrt(xp_sm(1,i+1)^2 + xp_sm(3,i+1)^2);
        y_sm(:,n+i+1) = sqrt(xp_sm(1,n+i+1)^2 + xp_sm(3,n+i+1)^2);
    end
    % Find measurement sigma point mean and covariance

    y_u = zeros(m,1);
    for i=1:2*n+1
        y_u = y_u + w_m(i)*y_sm(:,i);
    end
    Sy_u = zeros(m);
    for i=1:2*n+1
        Sy_u = Sy_u + w_c(i)*((y_sm(:,i)-y_u)*(y_sm(:,i)-y_u)');
    end
    Sy_u = Sy_u + Q;
    % Find cross covariance between x and y sigma points
    Sxy_u = zeros(n,m);
    for i=1:2*n+1
        Sxy_u = Sxy_u + w_c(i)*((x_sp(:,i)-mup_u)*(y_sm(:,i)-y_u)');
    end
    % Perform measurement update
    K_u = Sxy_u*inv(Sy_u);
    mu_u = mup_u + K_u*(y(:,t)-y_u);
    S_u = Sp_u - K_u*Sy_u*K_u';
    
    % Store results
    mup_Su(:,t) = mup_u;
    mu_Su(:,t) = mu_u;
    K_Su(:,t) = K_u;
    
    %% Plot results
    figure(1);clf; hold on;
    % True state
    plot(x(1,2:t),x(3,2:t), 'ro--')
    % EKF
    plot(mu_S(1,2:t),mu_S(3,2:t), 'bx--')
    % UKF
    plot(mu_Su(1,2:t),mu_Su(3,2:t), 'gx--')
    % EKF Ellipses
    mu_pos = [mu(1) mu(3)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    % UKF Ellipses
    mu_pos_u = [mu_u(1) mu_u(3)];
    S_pos_u = [S_u(1,1) S_u(1,3); S_u(3,1) S_u(3,3)];
    error_ellipse(S_pos_u,mu_pos_u,0.75);
    error_ellipse(S_pos_u,mu_pos_u,0.95);
    % Ground
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot([20 -1],[0 0],'b--')
    title('True state, EKF and UKF')
    axis([-5 20 -1 10])
    legend('True state', 'EKF', 'UKF')
    F(t-1) = getframe;
end
figure(2);clf; hold on;
e = sqrt((x(1,2:end)-mu_S(1,2:end)).^2+(x(3,2:end)-mu_S(3,2:end)).^2);
plot(T(2:end),e,'b', 'LineWidth', 1.5);
eu = sqrt((x(1,2:end)-mu_Su(1,2:end)).^2+(x(3,2:end)-mu_Su(3,2:end)).^2);
plot(T(2:end),eu,'g', 'LineWidth', 1.5);
title('Position Estimation Errors for EKF and UKF')
xlabel('Time (s)');
ylabel('X-Z Position Error (m)');
legend('EKF','UKF');

