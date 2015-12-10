% Particle filter example
clear;clc;

% Discrete time step
dt = 0.1;

% Prior
mu = 10; % mean (mu)
S = 1;% covariance (Sigma)

% Number of particles
M = 10;
% Prior - uniform over 5 15
X = 5+10*rand(1,M);
X0 = X;

%Motion model
A = 0.8;
B = 3;
R = 1;

% Measurement model
C = 1;
Q = 1;

% Simulation Initializations
Tf = 3;
T = 0:dt:Tf;
x = zeros(1,length(T)+1);
x(1) = mu+sqrt(S)*randn(1);
y = zeros(1,length(T));
u = y;

%% Main loop
tic;
for t=1:length(T)
    %% Simulation
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
    % Select a motion disturbance
    d = sqrt(Q)*randn(1);
    % Determine measurement
    y(t) = C*x(t+1) + d;

    
    %% Kalman Filter Estimation
    % Store prior
    mu_old = mu;
    S_old = S;

    % Prediction update
    mup = A*mu + B*u(t);
    Sp = A*S*A' + R;

    % Measurement update
    K = Sp*C'*inv(C*Sp*C'+Q);
    mu = mup + K*(y(t)-C*mup);
    S = (1-K*C)*Sp;
    
    %Store estimates
    mup_S(t)= mup;
    mu_S(t)= mu;

    %Particle filter estimation
    for m=1:M
        e = sqrt(R)*randn(1);
        Xp(m) = A*X(m) + B*u(t) + e;
        w(m) = normpdf(y(t),C*Xp(m),Q);
    end
    W = cumsum(w);
    for m=1:M
        seed = W(end)*rand(1);
        X(m) = Xp(find(W>seed,1));
    end
    muParticle = mean(X);
    SParticle = var(X);
    
    muP_S(t) = mean(X);
    SP_S(t) = var(X);
    
    %% Plot first time step results
    if (t == 1)
        L = 5;
        % Prior belief
        figure(1);clf; 
        subplot(2,1,1); hold on;
        z = [mu_old-L*sqrt(S_old):0.01:mu_old+L*sqrt(S_old)];
        plot(z,normpdf(z,mu_old,S_old),'b');
        plot([0 15],[1/15 1/15])
        title('Priors')
        axis([5 15 0 0.4])
        subplot(2,1,2); hold on;
        for m=1:M
            plot([X0(m);X0(m)],[0 1],'b')
        end
        axis([5 15 0 1])

        % Prediction step
        figure(2);clf; 
        subplot(2,1,1); hold on;
        plot(z,normpdf(z,mu_old,S_old),'b');
        z = [mup-L*sqrt(Sp):0.01:mup+L*sqrt(Sp)];
        plot(z,normpdf(z,mup,Sp),'r');
        title('Prior & Prediction')
        legend('Prior','Prediction')
        axis([0 15 0 0.4])
        subplot(2,1,2); hold on;
        for m=1:M
            plot([Xp(m);Xp(m)],[0 1],'r')
        end
        axis([0 15 0 1])

        % Measurement step
        figure(3);clf;
        subplot(2,1,1); hold on;
        plot(z,normpdf(z,mup,Sp),'r');
        z = [y(t)-L*sqrt(Q):0.01:y(t)+L*sqrt(Q)];
        plot(z,normpdf(z,y(t),Q),'g');
        z = [mu-L*sqrt(S):0.01:mu+L*sqrt(S)];
        plot(z,normpdf(z,mu,S), 'm');
        axis([0 15 0 0.6]);
        title('Prediction, Measurement & Belief')
        legend('Prediction','Measurement', 'Belief' )
        subplot(2,1,2); hold on;
        for m=1:M
            plot([X(m);X(m)],[0 1],'m')
        end
        axis([0 15 0 1]);
    end
   
end

toc
%Plot full trajectory results
figure(4);clf;hold on;
plot(T,x(2:end),'b')
%plot(T,y,'rx')
%plot(T,u,'g');
plot(T,mu_S,'r--')
plot(T,muP_S,'co--')
plot(T,2*ones(size(T)),'m--');
plot(T,10*ones(size(T)),'m--');
title('State and estimates, M=10')
legend('State', 'Kalman Estimate',  'Particle Estimate')

