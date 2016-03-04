function [mu, S, mup, K] = ekf(mu, S, y, motion_model, measurement_model, linearized_motion_model, linearized_measurement_model, Q, R, u)
% EFK - Run an iteration of an Extended Kalman Filter
%
% Inputs:
%
% mu - mean of the state
% S - covariance matrix
% y - measurement for the current time-step
% motion_model - reference to a function for the motion model
% measurement_model - reference to a function for the measurement model
% linearized_motion_model - reference to a function for the linearization of the motion model
% linearized_measurement_model - reference to a function for the linearization of the measurement model
% Q - measurement disturbance model
% R - motion disturbance model
% u - system input at the current timestep
%
% Outputs:
%
% mu - updated mean
% S - updated covariance matrix
% mup - predicted mean
% K - Kalman gain

    if ~exist('u', 'var')
        u = 0;
    end

    %%% Prediction update
    
    % Propagate mu through the nonlinear motion model
    mup = motion_model(mu, u);
    
    % Linearize motion model at the predicted mean
    G = linearized_motion_model(mup, u);
    
    % Compute predicted covariance
    Sp = G*S*G' + R;

    
    %%% Measurement update
    
    % Linearize measurement model at the predicted mean
    H = linearized_measurement_model(mup, u);

    % Compute Kalman gain
    K = Sp*H'*inv(H*Sp*H'+Q);
    
    % Update mean using the nonlinear measurement model
    mu = mup + K*(y-measurement_model(mup, u));
    
    % Update the covariance based on the measurement model
    S = (eye(length(mu))-K*H)*Sp;
end