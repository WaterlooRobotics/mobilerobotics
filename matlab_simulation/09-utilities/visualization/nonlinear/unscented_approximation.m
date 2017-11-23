function [X, Y, muU, yNU, pyNU] = unscented_approximation(L, S, mu)
    n = 1;
    alpha = 2;
    kappa = 1;
    beta = 2;
    lambda = alpha^2 * (n + kappa) - n;

    X(:,1) = mu;
    Y(:,1) = atan(X(:,1) + 0.5);
    nlS = sqrt((n + lambda) * S);
    for i = 1:n
       X(:, i + 1) =  mu + nlS(:, i);
       Y(:, i + 1) = atan(X(:, i + 1) + 0.5);
       X(:, n + i + 1) =  mu - nlS(:, i);
       Y(:, n + i + 1) = atan(X(:, n + i + 1) + 0.5);
    end
    wm(1) = lambda / (n + lambda);
    wc(1) = wm(1) + (1 - alpha^2 + beta) + (1 - alpha^2 + beta);
    wm(2:2 * n + 1) = 1 / (2 * (n + lambda));
    wc(2:2 * n + 1) = 1 / (2 * (n + lambda));

    muU = zeros(n,1);
    SU = zeros(n,n);
    for i = 1:2 * n + 1
        muU = muU + wm(i) * Y(:, i);
    end
    for i = 1:2 * n + 1
        SU = SU + wc(i) * (Y(:, i) - muU) * (Y(:, i) - muU)';
    end
    yNU = [muU-L*sqrt(SU):0.01:muU+L*sqrt(SU)];
    pyNU = normpdf(yNU, muU, sqrt(SU));
end

