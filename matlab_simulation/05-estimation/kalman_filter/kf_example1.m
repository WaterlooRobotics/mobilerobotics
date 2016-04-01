% Kalman Filter Example 1: Temperature Control
% 'Estimation 1' topic slide 48
% State : Current temperature difference with outside
% Motion : Decaying temperature + furnace input + disturbances
% Measurement : Current temperature difference
% Controller : Bang bang controller

clear all
clc
close all

% Example number
example = 1;

% Discrete time step
dt = 0.1;

% Prior
mu = 10; % mean (mu)
S = 1;   % covariance (Sigma)

% Get matrices for motion model and measurement model
[A,B,R,n] = motion_model(example,dt);
[C,D,Q,m] = measurement_model(example);

% Store in a structure (State Space Model [ssm])
ssm.A = A;
ssm.B = B;
ssm.C = C;
ssm.D = D;
ssm.R = R;
ssm.Q = Q;
ssm.n = n;
ssm.m = m;

% Simulation Initializations
Tf = 3;
T = 0:dt:Tf;
x = zeros(1,length(T)+1);
x(1) = mu+sqrt(S)*randn(1);
y = zeros(m,length(T));
u = y;

% Frequency variable used for multirate kalman filter(eg 3). 
% Not used in eg 1 & 2
freq = 0;

%% Main loop
for t=1:length(T)
    % Simulation ----------------------------------------------------
    % Select control action
    if (t>1)
        u(t)=u(t-1);
    end
    if (mu > 10)
        u(t) = 0;
    elseif (mu < 2);
        u(t) = 1;
    end
    
    % Select a motion disturbance
    e = sqrt(R)*randn(1);
    
    % Update state
    x(t+1) = A*x(t)+ B*u(t) + e;

    % Take measurement
    % Select a measurement disturbance
    d = sqrt(Q.Q)*randn(1);
    
    % Determine measurement
    y(t) = C*x(t+1) + d;
    
    % Store prior --------------------------------------------------
    mu_old = mu;
    S_old = S;
    
    % Kalman Filter Estimation
    [mu,S,mup,Sp,K] = kalman_filter(ssm,mu,S,u(:,t),y(:,t),example,t,freq);
    
    % Store estimates
    mup_S(t)= mup;
    mu_S(t)= mu;
    
    % Plot first time step results
    if (t == 1)
        L = 5;
        
        % Prior belief
        figure(1);clf; subplot(2,2,1); hold on;
        temperature = [mu_old-L*sqrt(S_old):0.01:mu_old+L*sqrt(S_old)];
        plot(temperature,normpdf(temperature,mu_old,S_old),'b');
        title('Prior')
        ylabel('Probability')
        xlabel('Temperature')
        
        % Prediction step
        %figure(2);clf; 
        subplot(2,2,2); hold on;
        plot(temperature,normpdf(temperature,mu_old,S_old),'b');
        temperature = [mup-L*sqrt(Sp):0.01:mup+L*sqrt(Sp)];
        plot(temperature,normpdf(temperature,mup,Sp),'r');
        title('Prior & Prediction')
        legend('Prior','Prediction')
        ylabel('Probability')
        xlabel('Temperature')
        
        % Measurement step
        %figure(3);clf; 
        subplot(2,2,3); hold on;
        plot(temperature,normpdf(temperature,mup,Sp),'r');
        temperature = [y(t)-L*sqrt(Q.Q):0.01:y(t)+L*sqrt(Q.Q)];
        plot(temperature,normpdf(temperature,y(t),Q.Q),'g');
        temperature = [mu-L*sqrt(S):0.01:mu+L*sqrt(S)];
        plot(temperature,normpdf(temperature,mu,S), 'm');
        axis([-10 20 0 .35]);
        title('Prediction, Measurement & Belief')
        legend('Prediction','Measurement', 'Belief' )
        ylabel('Probability')
        xlabel('Temperature')
    end
end

% Plot full trajectory results
% figure(2);clf;
subplot(2,2,4); hold on;
plot(T,x(2:end),'b')
plot(T,y,'rx')
plot(T,mup_S,'c--')
plot(T,mu_S,'r--')
plot(T,u,'g');
plot(T,2*ones(size(T)),'m--');
plot(T,10*ones(size(T)),'m--');
title('State and estimates')
legend('State','Measurement', 'Prediction', 'Estimate', 'Input')
ylabel('Temperature')
xlabel('Time')