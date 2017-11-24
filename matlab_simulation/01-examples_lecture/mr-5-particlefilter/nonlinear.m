%% Comparison of linear and nonlinear transformations
% Original Gaussian distribution
mu = 0;  % mean (mu)
S = 1;  % covariance (Sigma)
L = 5;
x = [mu-L*sqrt(S):0.01:mu+L*sqrt(S)];  % x points
px = normpdf(x,mu,S);  % p(x) 
n = 5000000;  % Number of samples to use
xS = sqrt(S)*randn(n,1);  % Samples of original distribution

% Linear transformation and resulting Gaussian distribution
[yL, muL, SL, pyL] = linear_transform(x, mu, S);

% Nonlinear transformation
[yN, YN, pyN, muN, yNG, pyNG] = nonlinear_transform(x, xS, n, L);

% Linearized propagation of mean and covariance (a la EKF)
[muLN, SLN, yLN, pyLN] = ekf_approximation(L, S, mu);

% Unscented approximation to resulting distribution
[X, Y, muU, yNU, pyNU] = unscented_approximation(L, S, mu);


%% Display results
plot_linear_transform(1, x, px, yL, pyL);
plot_nonlinear_transform(2, x, px, yN, pyN, YN, pyNG, yNG, muN);
plot_resulting_distributions(3,  X, Y, yN, YN, pyN, yNG, pyNG, yLN, pyLN, yNU, pyNU, muN, muU, muLN);

