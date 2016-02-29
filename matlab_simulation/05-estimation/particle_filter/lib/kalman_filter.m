function [mup, mu, Sp, S] = kalman_filter(t, A, B, C, R, Q, y, u, mu, S)
    % prediction update
    mup = A * mu + B * u(t);
    Sp = A * S * transpose(A) + R;

    % measurement update
    K = Sp * transpose(C) * inv(C * Sp * transpose(C) + Q);
    mu = mup + K * (y(t) - C * mup);
    S = (1 - K * C) * Sp;
end

