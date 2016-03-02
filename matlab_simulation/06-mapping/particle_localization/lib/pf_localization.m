function [X] = pf_localization(t, dt, meas, n, D, R, Q, d, X, Xp, mf, y, u)
    [RE, Re] = eig(R);
    w = zeros(1, D);

    % sample
    for dd = 1:D
        e = RE * sqrt(Re) * randn(n, 1);
        Xp(:, dd) = [X(1, dd) + u(1, t) * cos(X(3, dd)) * dt;
                    X(2, dd) + u(1, t) * sin(X(3, dd)) * dt;
                    X(3, dd) + u(2, t) * dt] + e;
        switch(meas)
            case 1  % 1 - range
                r = range(mf(1, t), Xp(1, dd), mf(2, t), Xp(2, dd));
                hXp = r + d;
            case 2  % 2 - bearing
                b = bearing(mf(1, t), mf(2, t), Xp(1, t), Xp(2, t), Xp(3, t));
                hXp = b + d;
            case 3  % 3 - both
                r = range(mf(1, t), Xp(1, dd), mf(2, t), Xp(2, dd));
                b = bearing(mf(1, t), mf(2, t), Xp(1, dd), Xp(2, dd), Xp(3, dd));
                hXp = [r; b] + d;
        end
        w(dd) = max(1e-8, mvnpdf(y(:, t), hXp, Q));
    end
    W = cumsum(w);

    % importance re-sample
    for dd = 1:D
        seed = max(W) * rand(1);
        X(:, dd) = Xp(:, find(W > seed, 1));
    end
end

