clear all
clc
close all

% Test Example 1

% Discrete time step
dt = 0.1;

% Prior
mu = 10; % mean (mu)
S = 1;% covariance (Sigma)

%Motion model
A = 0.8;
B = 3;
R = 2;

% Measurement model
C = 1;
Q = 4;

% Simulation Initializations
Tf = 3;
T = 0:dt:Tf;
x = zeros(1,length(T)+1);
x(1) = mu+sqrt(S)*randn(1);
y = zeros(1,length(T));
u = y;

for t=1:length(T)
    new_state = kalman_filter()
end
