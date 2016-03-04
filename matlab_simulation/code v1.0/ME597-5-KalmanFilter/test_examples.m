% This file runs different examples of the Kalman filter based on the
% selection

clear all
clc
close all

% Test Example 1,2 or 3 by changing number
example = 1;
% example = 2;
% example = 3;

% Discrete time step and simulation runtime
dt = 0.1;
Tf = 3;
T = 0:dt:Tf;

% Get Prior, motion model and measurement model
[mu, S] = prior(example);
[A,B,R] = motion_model(example,dt);
[C,D,Q] = measurement_model(example);

I

% Simulation Initializations
x = zeros(1,length(T)+1);
x(1) = mu+sqrt(S)*randn(1);
y = zeros(1,length(T));
u = y;
n = length(A(1,:));
m = length(C(:,1));

% Clear unwanted variables before iteration
%clear sysc A B C D Tf dt

%%
for t=2:length(T)
    
    % Apply control
    u(:,t) = control_input(example, mu, t, u(:,t-1));
    
    % Motion disturbance
    e = sqrt(R)*randn(n,1);
    
    % Update State
    x(:,t) = Ad*x(:,t-1)+ Bd*u(:,t) + e;
    
    % Take measurement
    % Select a measurement disturbance
    d = sqrt(Q)*randn(m,1);
    
    % Determine measurement
    y(:,t) = Cd*x(:,t) + d;
    
    % Store prior
    mu_old = mu;
    S_old = S;
    
    % Save old mean and covariance
    [mu_S(:,t),cov_S(:,t),mup_S(:,t),K(:,t)] = kalman_filter(sysd,mu,S,R,u(:,t),y(:,t),Q);
    
    % Store estimates
    K_S(:,t) = [K(:,1); K(:,2)];
end
