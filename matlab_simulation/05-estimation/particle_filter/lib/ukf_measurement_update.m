function [K_u, mu_u, S_u] = ukf_measurement_update(t, Ad, Q, lambda, mup_u, Sp_u, x_sp, y, w_m, w_c)
    n = length(Ad(1,:));
    m = length(Q(:,1));

    nSp_u = sqrtm((n + lambda) * Sp_u);
    xp_sm(:, 1) = mup_u;
    y_sm(:, 1) = sqrt(xp_sm(1, 1)^2 + xp_sm(3, 1)^2);
    
    for i=1:n
        % Sigma points prior to measurement
        xp_sm(:, i + 1) = mup_u + transpose(nSp_u(i, :));
        xp_sm(:, n + i + 1) = mup_u - transpose(nSp_u(i, :));
        
        % Measurement model applied to sigma points
        y_sm(:, i + 1) = sqrt(xp_sm(1, i + 1)^2 + xp_sm(3, i + 1)^2);
        y_sm(:, n + i + 1) = sqrt(xp_sm(1, n + i + 1)^2 + xp_sm(3, n + i + 1)^2);
    end
    
    % Find measurement sigma point mean and covariance
    y_u = zeros(m, 1);
    for i = 1:(2 * n + 1)
        y_u = y_u + w_m(i) * y_sm(:, i);
    end
    Sy_u = zeros(m);
    for i = 1:(2 * n + 1)
        Sy_u = Sy_u + w_c(i) * ((y_sm(:, i) - y_u) * (y_sm(:, i) - y_u)');
    end
    Sy_u = Sy_u + Q;
    
    % Find cross covariance between x and y sigma points
    Sxy_u = zeros(n,m);
    for i=1:(2 * n + 1)
        Sxy_u = Sxy_u + w_c(i) * ((x_sp(:, i) - mup_u) * (y_sm(:, i) - y_u)');
    end
    
    % Perform measurement update
    K_u = Sxy_u * inv(Sy_u);
    mu_u = mup_u + K_u * (y(:,t)-y_u);
    S_u = Sp_u - K_u * Sy_u * K_u';
end

