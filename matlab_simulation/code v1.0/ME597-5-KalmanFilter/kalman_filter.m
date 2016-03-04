function [mean_,covariance,mean_predicted, kalman_gain] = kalman_filter(motion_model,mean_,covariance,R,measurement_model,measurement,Q)

% Performs one iteration of Kalman Filtering

% Inputs:
%       motion_model       : Function reference for motion model
%       mean_              : Mean of current state
%       covariance         : Covariance of current state
%       R                  : Motion disturbance
%       measurement_model  : Function reference for measurement model
%       measurement        : Measurment of current time step 
%       Q                  : Measurement disturbance

% Outputs:
%       mean_predicted     : Predicted mean after motion update
%       kalman_gain        : Kalman gain for current time step
%       mean_              : Final estimated mean after iteration
%       covariance         : Final estimated covariance after iteration 

% Prediction Step:

% Measurement Step:

% Initialize
n = length(Ad(1,:));

% Update State
x_new = Ad*x+ Bd*u + e;

% Determine measurement
y = Cd*x_new + d;

% Kalman filter estimation
% Prediction Update
mup = Ad*mu + Bd*u;
Sp = Ad*S*Ad' + R;

% Measurement Update
K = Sp*Cd'*inv(Cd*Sp*Cd'+Q);
mu = mup + K*(y-Cd*mup);
S = (eye(n)-K*Cd)*Sp;
