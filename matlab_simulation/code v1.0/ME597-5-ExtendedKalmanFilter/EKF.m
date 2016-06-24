function [mu,S,K,mup] = EKF(mu,S,w,v,dt,y,motion_model,sensor_model,linearize_motion_model,linearize_sensor_model,R,Q)
%% Extended Kalman Filter.
%This function takes mu, Covariance, motion and sensor model and their derivaties as input
%and returns Kalman Gain, Updated mu & Covariance and predicted mu.


%% Extended Kalman Filter Estimation
    n=length(mu);
    %y=sensor_model(mu);

    % Linearization
    Gt = linearize_motion_model(mu,v,dt);
    Ht = linearize_sensor_model(mu);
    
    % Prediction
    mup = motion_model(mu,w,v,dt);
    Sp = Gt*S*Gt' + R;
    
    % update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup + K*(y-sensor_model(mup));
    S = (eye(n)-K*Ht)*Sp;

end