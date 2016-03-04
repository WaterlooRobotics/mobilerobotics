function [mean, covariance] = prior(example)

% This function provides a prior

% For exmaple 1
if example == 1
    mean = 10; % mean (mu)
    covariance = 1; % ovariance (Sigma)
end

% For exmaple 2
if example == 2
    mean = zeros(4,1); % mean (mu)
    covariance = 1*eye(4);% covariance (Sigma)
    %covariance = 0.1*eye(4);% covariance (Sigma)
    %covariance = 0.01*eye(4);% covariance (Sigma)
end

% For exmaple 3
if example == 3
    mean = zeros(4,1); % mean (mu)
    covariance = 0.01*eye(4);% covariance (Sigma)
end