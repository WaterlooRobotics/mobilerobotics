function [mu, S, dt, Tf] = prior(example)

% This function provides a prior for the mean (mu), covariance (S) and time
% step (dt)

% For exmaple 1
if example == 1
    mu = 10; % mean (mu)
    S = 1; % covariance (Sigma)
    dt = 0.1;
    Tf = 3;
end

% For exmaple 2
if example == 2
    mu = zeros(4,1); % mean (mu)
    S = 1*eye(4);% covariance (Sigma)
    %covariance = 0.1*eye(4);% covariance (Sigma)
    %covariance = 0.01*eye(4);% covariance (Sigma)
    dt = 0.1;
    Tf = 10;
end

% For exmaple 3
if example == 3
    mu = zeros(4,1); % mean (mu)
    S = 0.01*eye(4);% covariance (Sigma)
    dt = 0.01;
    Tf = 1;
end