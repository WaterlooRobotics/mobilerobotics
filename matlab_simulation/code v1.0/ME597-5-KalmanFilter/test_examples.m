% This file runs different examples of the Kalman filter based on the
% selection

clear all
clc
close all

% Test Example 1,2 or 3 by changing number 
% example = 1;
% example = 2;
 example = 3;

% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('multiratekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Get Prior, motion model and measurement model
[mu,S,dt,Tf] = prior(example);
[A,B,R,n] = motion_model(example,dt);
[C,D,Q,m] = measurement_model(example);

% Simulation runtime
T = 0:dt:Tf;
T_len = length(T);

% Simulation Initializations
[x,y,u,mup_S,mu_S] = initialization(example,T_len,n,m,S);

% Clear unwanted variables before iteration
% clear sysc A B C D Tf dt

% Get first control
t = 1;
u(:,1) = control_input(example, mu, T(t), u(:,t));
%%
for t=2:length(T)
    
    % Apply control
    u(:,t) = control_input(example, mu, T(t), u(:,t-1));
    
    % Simulation
    [x(:,t), y(:,t)] = simulation(A,B,C,D,R,Q,x(:,t-1),t,u(:,t),example);
    
    % Store prior
    % mu_old = mu;
    % S_old = S;
    
    % Save old mean and covariance
    [mu,S,mup,K] = kalman_filter(A,B,C,D,mu,S,R,u(:,t),y(:,t),Q,example,t);
    
    mu_S(:,t) = mu;
    mup_S(:,t) = mup;
    
    % Store estimates
    K_S(:,t) = [K(:,1); K(:,2)];
    
    % Test plot
    plot_filter(x,y,t,mu_S,mu,S,example,makemovie,vidObj);
    
end

if makemovie
    close(vidObj);
end

figure(2);clf;
plot(T,K_S');
title('Kalman gains as a function of time')

figure(3);clf;
plot(T(2:t),x(:,2:t)-mu_S(:,2:t))