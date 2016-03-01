addpath('./lib/importance_sampling_example')

%% Importance Sampling graphic
clear;
clc;

%% Generate two interesting distributions
% First distribution
L = 5;
mu = 0;  % mean (mu)
S = 0.5;  % covariance (Sigma)
x = [mu-L*sqrt(S):0.005:mu+L*sqrt(S)];  % x points
gx = normpdf(x, mu, S);  % p(x)
gx = gx + 0.1;  % Add uniform base probability
gx = gx / sum(gx);  % Normalize
Gx = cumsum(gx);

% Second distribution
mu1 = -1;
S1 = .25;
fx1 = normpdf(x, mu1, S1);
mu2 = -0;
S2 = 0.5;
fx2 = normpdf(x, mu2, S2);
fx = fx1 + fx2 + 0.1;  % form f(x)
fx = fx / sum(fx);  % normalize
Fx = cumsum(fx);

%% Importance sampling
M = 20000;  % number of particles
[indP, xP, xPind] = is_sampling(M, Gx, x);
[wx] = is_weighting(M, x, gx, fx, xP);
[xPnew, xPindnew] = is_resampling(M, xP, wx);

%% Plot
plot_initial_sample(1, M, x, fx, gx, xP)
plot_sample_weights(2, M, x, fx, gx, xP, wx)
plot_distribution_importance(3, M, x, fx, gx, xPnew);
