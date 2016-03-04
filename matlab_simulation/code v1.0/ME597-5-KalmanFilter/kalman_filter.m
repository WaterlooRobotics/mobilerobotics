function [mean_,covariance,mean_predicted, kalman_gain] = kalman_filter(sysd,mean_,covariance,R,control,measurement,Q)

% Performs one iteration of Kalman Filtering

% Inputs:
%       motion_model       : Function reference for motion model
%       mean_              : Mean of current state
%       covariance         : Covariance of current state
%       R                  : Motion disturbance
%       measurement_model  : Function reference for measurement model
%       measurement        : Measurment of current time step 
%       Q                  : Measurement disturbance
%       example            : Which example we want to test

% Outputs:
%       mean_predicted     : Predicted mean after motion update
%       kalman_gain        : Kalman gain for current time step
%       mean_              : Final estimated mean after iteration
%       covariance         : Final estimated covariance after iteration 

% Get A,B,C,D matrices for motion prediction
A = sysd.A;
B = sysd.B;
C = sysd.C;
D = sysd.D;

% Prediction Step:

% Get dimension of state
n = length(A(1,:));

% Predict new mean based on motion
mup = A*mean_ + B*control;

% Predict new covariance based on motion
Sp = A*covariance*A' + R;

% Measurement Step:

% Calculate kalman gain
kalman_gain = Sp*C'*inv(C*Sp*C'+Q);

% Get new mean of state after measurement
mean_ = mup + kalman_gain*(measurement-C*mup);

% Get new covaraince of state after measurement
covariance = (eye(n)-kalman_gain*C)*Sp;
