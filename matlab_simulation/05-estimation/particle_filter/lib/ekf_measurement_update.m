function [K, mu, S] = ekf_measurement_update(t, Ad, Ht, Q, y, mup, Sp)
    n = length(Ad(1,:));
    
    % measurement update
    K = Sp * transpose(Ht) * inv(Ht * Sp * transpose(Ht) + Q);
    mu = mup + K * (y(:,t) - sqrt(mup(1)^2 + mup(3)^2));
    S = (eye(n) - K * Ht) * Sp;
end