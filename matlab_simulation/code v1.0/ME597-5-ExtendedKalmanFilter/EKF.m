function [mu,S,K,mup] = EKF(g,Gt,Ht,S,Y,sensor_model,R,Q)
%% Extended Kalman Filter.
%This function takes Covariance, motion and sensor model and their derivaties and noise as input
%and returns Kalman Gain, Updated mu & Covariance and predicted mu.

%% Extended Kalman Filter Estimation
    % Linearization
    % taking linearized models as input so as to make this function more generic
    
    % Prediction
    mup = g;
    n=length(mup);
    Sp = Gt*S*Gt' + R;
    
    % update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup + K*(Y-sensor_model(mup));
    S = (eye(n)-K*Ht)*Sp;
end