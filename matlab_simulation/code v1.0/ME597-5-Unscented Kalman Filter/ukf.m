% Unscented Kalman filter example
clear;clc;
% reset(RandStream.getDefaultStream);

% Discrete time step
dt = 0.1;

% Initial State
x0 = [20 3]';
v = [-0.2 0]';

% Prior
mu = [22 6]'; % mean (mu)
S = 1*eye(2);% covariance (Sigma)
mu_u = mu;
S_u = S;

% Discrete motion model
Ad = [1 0; 0 1];
Bd = [1 0; 0 1];

R = [.001 0 ; 0 .001];
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
alpha = 0.5;
kappa = 3-n;
beta = 2;
%functionalize weighted values
[w_m,w_c,lambda] = WeightedValues(kappa,alpha,beta,n);


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = Ad*x(:,t-1) + Bd*v + e;

    % Take measurement
    % Select a motion disturbance
    d = sqrt(Q)*randn(m,1);
    % Determine measurement
    y(:,t) = sqrt(x(1,t)^2 + x(2,t)^2) + d;


    %% Extended Kalman Filter Estimation
    [mup,mu,S,K] = EKF (Ad,Bd,0,0,y(:,t),v,mu,S,R,Q,n);
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = K;

    %% Unscented Kalman Filter Estimation
    % Prediction update
    [xp_sp,x_sp,nS_u] = Update (n,Ad,Bd,v,lambda,S_u,mu_u,1);
    [xp_sp,x_sp] = SelectSigPoints(n,Ad,Bd,v,mu_u,nS_u,xp_sp,x_sp,1);
    
    mup_u = zeros(n,1);
    Sp_u = zeros(n);
    [mup_u,Sp_u] = ExtractMuSig (n,mup_u,w_m,x_sp,Sp_u,w_c,R);
      
    % Measurement update
    [xp_sm,y_sm,nSp_u] = Update (n,0,0,0,lambda,Sp_u,mup_u,2);
    [xp_sm,y_sm] = SelectSigPoints (n,0,0,0,mup_u,nSp_u,xp_sm,y_sm,2);
    % Find measurement sigma point mean and covariance

    y_u = zeros(m,1);
    Sy_u = zeros(m);
    [y_u,Sy_u] = ExtractMuSig (n,y_u,w_m,y_sm,Sy_u,w_c,Q);
    
    % Find cross covariance between x and y sigma points
    Sxy_u = zeros(n,m);
    [Sxy_u] = CrossCovariance (Sxy_u,n,w_c,x_sp,mup_u,y_sm,y_u);

    % Perform measurement update
    [K_u,mu_u,S_u] = CovarianceUpdate (t,Sxy_u,Sy_u,Sp_u,mup_u,y,y_u);

    % Store results
    mup_Su(:,t) = mup_u;
    mu_Su(:,t) = mu_u;
    K_Su(:,t) = K_u;
    
    %% Plot results
    PlotData(t,x,mu_u,mu_S,mu_Su,mu,S,S_u);
end
%%UKF and EFK error comparison
CompareEKF_UKF (T,x,mu_S,mu_Su);
