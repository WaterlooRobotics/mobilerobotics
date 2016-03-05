function [mu,S,mup, K] = kalman_filter(model_params,mu,S,u,y,example,t)

% Performs one iteration of Kalman Filtering

% Inputs:
%       model_params  : Parameters for motion model
%       mu            : Mean of current state
%       S             : Covariance of current state
%       y             : Measurment of current time step
%       example       : Which example we want to test
%       t             : current time step
% Outputs:
%       mu            : Final estimated mean after iteration
%       S             : Final estimated covariance after iteration
%       mup           : Predicted mean after motion update
%       K             : Kalman gain for current time step 

A = model_params.A;
B = model_params.B;
C = model_params.C;
D = model_params.D;
R = model_params.R;
Q = model_params.Q;
n = model_params.n;
m = model_params.m;

% Prediction Step: ------------------------------------------------
% Predict new mean based on motion
mup = A*mu + B*u;
% Predict new covariance based on motion
Sp = A*S*A' + R;

% Measurement Step: ------------------------------------------------
if example == 3 % exmaple 3 is a multirate kalman filter
    Qp = Q.Qp;
    Qv = Q.Qv;
    Cp = C.Cp;
    Cv = C.Cv;
    if (mod(t,10) == 0)
        % Calculate kalman gain
        K = Sp*Cp'*inv(Cp*Sp*Cp'+Qp);
        % Get new mean of state after measurement
        mu = mup + K*(y-Cp*mup);
        % Get new covaraince of state after measurement
        S = (eye(n)-K*Cp)*Sp;
    else
        K = Sp*Cv'*inv(Cv*Sp*Cv'+Qv);
        mu = mup + K*(y([2 4],:)-Cv*mup);
        S = (eye(n)-K*Cv)*Sp;
    end
    
else % for example 1 and 2
    K = Sp*C'*inv(C*Sp*C'+Q.Q);
    mu = mup + K*(y-C*mup);
    S = (eye(n)-K*C)*Sp;
end