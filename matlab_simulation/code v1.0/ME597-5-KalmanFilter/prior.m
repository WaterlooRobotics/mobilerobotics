function [mean, coveriance] = prior(example)

% This function provides a prior

if example == 1
    mean = 10; % mean (mu)
    covariance = 1;
end

if example == 2
    mean = zeros(4,1); % mean (mu)
    covariance = 1*eye(4);% covariance (Sigma)
    %covariance = 0.1*eye(4);% covariance (Sigma)
    %covariance = 0.01*eye(4);% covariance (Sigma)
end

if example == 3
    mean = zeros(4,1); % mean (mu)
    covariance = 0.01*eye(4);% covariance (Sigma)
end