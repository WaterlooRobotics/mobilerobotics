function [muParticle, SParticle, X, Xp] = pf(t, M, A, B, C, X, R, Q, y, u)
    % sample
    for m = 1:M
        e = sqrt(R) * randn(1);
        Xp(m) = A * X(m) + B * u(t) + e;
        w(m) = normpdf(y(t), C * Xp(m), Q);
    end

    % re-sample
    W = cumsum(w);
    for m=1:M
        seed = W(end) * rand(1);
        X(m) = Xp(find(W > seed, 1));
    end

    muParticle = mean(X);
    SParticle = var(X);
end

