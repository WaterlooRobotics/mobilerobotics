function [mup_u, Sp_u, x_sp, xp_sp] = ukf_prediction_update(Ad, S_u, mu_u, R, lambda, w_m, w_c)
    n = length(Ad(1,:));
    
    % Prediction update
    nS_u = sqrtm((n + lambda) * S_u);
    xp_sp(:, 1) = mu_u;
    x_sp(:, 1) = Ad * xp_sp(:, 1);
    
    for i = 1:n
        % Sigma points prior to propagation
        xp_sp(:, i + 1) = mu_u + nS_u(:, i);
        xp_sp(:, n + i + 1) = mu_u - nS_u(:, i);
        
        % Sigma points after propagation
        x_sp(:, i + 1) = Ad * xp_sp(:, i + 1);
        x_sp(:, n + i + 1) = Ad * xp_sp(:, n + i + 1);
    end

    % calculate mean
    mup_u = zeros(n, 1);
    for i = 1:2 * n+1
        mup_u = mup_u + w_m(i) * x_sp(:, i);
    end
    
    % calculate covariance
    Sp_u = zeros(n);
    for i = 1:(2 * n + 1)
        Sp_u = Sp_u + w_c(i) * ((x_sp(:, i) - mup_u) * transpose((x_sp(:, i) - mup_u)));
    end
    Sp_u = Sp_u + R;  % add motion variance as well
end

