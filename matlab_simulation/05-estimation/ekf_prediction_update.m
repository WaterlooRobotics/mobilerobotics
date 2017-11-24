function [mup, Sp] = ekf_prediction_update(Ad, R, mu, S)
    % prediction update
    mup = Ad * mu;
    Sp = Ad * S * transpose(Ad) + R;
end