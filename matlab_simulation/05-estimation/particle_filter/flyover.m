% Unscented Kalman filter example
addpath('./lib')
addpath('./lib/flyover')
clear;
clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('ekflocalization.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.1;

% Initial State
x0 = [15 -2 3]';

% Prior
mu = [16 -1.8 4]'; % mean (mu)
S = 1 * eye(3); % covariance (Sigma)
mu_u = mu;
S_u = S;

% Discrete motion model
Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];

R = [.001 0 0; 0 .001 0 ; 0 0 .001];
[RE, Re] = eig (R);

% Measurement model defined below
Q = [.01];

% Simulation Initializations
Tf = 8;
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
lambda = alpha^2 * (n + kappa) - n;
w_m(1) = lambda / (n + lambda);
w_c(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);
w_m(2:2 * n + 1) = 1/(2 * (n + lambda));
w_c(2:2 * n + 1) = 1/(2 * (n + lambda));

% Particle Filter Parameters
% %Number of particles
I = 5000;
% % Prior
X = randn(3, I);
X(1,:) = X(1, :) + mu(1);
X(2,:) = X(2, :) + mu(2);
X(3,:) = X(3, :) + mu(3);
X0 = X;


%% Main loop
for t = 2:length(T)
    % update state
    e = RE * sqrt(Re) * randn(n,1);
    x(:,t) = Ad * x(:,t - 1) + e;

    % take measurement
    d = sqrt(Q) * randn(m,1);
    y(:, t) = sqrt(x(1, t)^2 + x(3, t)^2) + d;

    
    %% Extended Kalman Filter Estimation
    % Prediction update
    [mup, Sp] = ekf_prediction_update(Ad, R, mu, S);
    % Linearization
    Ht = [(mup(1))/(sqrt(mup(1)^2 + mup(3)^2)) 0 (mup(3))/(sqrt(mup(1)^2 + mup(3)^2))];
    % Measurement update
    [K, mu, S] = ekf_measurement_update(t, Ad, Ht, Q, y, mup, Sp);

    % Store EKF results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = K;

    
    %% Unscented Kalman Filter Estimation
    % Prediction update
    [mup_u, Sp_u, x_sp, xp_sp] = ukf_prediction_update(Ad, S_u, mu_u, R, lambda, w_m, w_c);
    % Measurement update
    [K_u, mu_u, S_u] = ukf_measurement_update(t, Ad, Q, lambda, mup_u, Sp_u, x_sp, y, w_m, w_c);

    % Store UKF results
    mup_Su(:, t) = mup_u;
    mu_Su(:, t) = mu_u;
    K_Su(:, t) = K_u;

    %% Particle Filter
    [muParticle, SParticle, X] = pf_nonlinear(t, I, Ad, X, R, Q, y);
    
    % store PF results
    muP_S(:, t) = muParticle;
    SP_S(:, :, t) = SParticle;

    
    % plot results
    plot_ekf_ufk_pf(1, t, x, mu, mu_S, mu_Su, muP_S, S, S_u, X);

    if (makemovie)
        writeVideo(vidObj, getframe(gca));
    end

end

if (makemovie)
    close(vidObj);
end

% plot errors
plot_errors(2, T, x, mu_S, mu_Su, muP_S);



