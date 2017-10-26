function [yL, muL, SL, pyL] = linear_transform(x, mu, S)
    % Linear transformation
    A = 2;
    b = -2;
    yL = A * x + b;

    % Resulting Gaussian distribution
    muL = A * mu + b;
    SL = A * S * A';
    pyL = normpdf(yL, muL, SL);
end