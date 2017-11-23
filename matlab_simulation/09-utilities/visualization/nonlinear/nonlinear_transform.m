function [yN, YN, pyN, muN, yNG, pyNG] = nonlinear_transform(x, xS, n, L)
    yN = atan(x + 0.5);  % Nonlinear transformation of pdf
    yNS = atan(xS + 0.5);  % Nonlinear transformation of samples

    % Resulting distribution
    m = 100; % Number of bins to use
    [d, YN] = hist(yNS, m); % Histogram of results
    binw = (max(YN) - min(YN)) / (m - 1); % Bin width
    pyN = d / n / binw; % Resulting distribution

    % Gaussian approximation to resulting distribution
    muN = mean(yNS); % Mean of samples of transformed distribution
    SN = var(yNS); % Covariance of samples of transformed distribution
    yNG = [muN-L*sqrt(SN):0.01:muN+L*sqrt(SN)];
    pyNG = normpdf(yNG, muN, sqrt(SN));
end

