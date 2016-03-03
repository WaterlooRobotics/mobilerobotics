% Inputs: prior state prediction, prior covariance matrix, measurement,
% motion model, measurement model, Q, and R

function mu, S = ekf(mu, S, y, motion_model, measurement_model, linearized_motion_model, linearized_measurement_model, Q, R)

    % Prediction update
    mup = motion_model(mu);
    G = linearized_motion_model(mup);
    Sp = G*S*G' + R;

    % Linearize measurement model at the predicted mean
    H = linearized_measurement_model(mup);

    % Measurement update
    K = Sp*H'*inv(H*Sp*H'+Q);
    mu = mup + K*(y-measurement_model(mup));
    S = (eye(length(mu))-K*H)*Sp;
end