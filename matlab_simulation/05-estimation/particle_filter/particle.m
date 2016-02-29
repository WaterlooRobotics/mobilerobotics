pkg load statistics

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

function [mup, mu, Sp, S] = kalman_filter(t, A, B, C, R, Q, y, u, mu, S)
    % prediction update
    mup = A * mu + B * u(t);
    Sp = A * S * transpose(A) + R;

    % measurement update
    K = Sp * transpose(C) * inv(C * Sp * transpose(C) + Q);
    mu = mup + K * (y(t) - C * mup);
    S = (1 - K * C) * Sp;
end

function [muParticle, SParticle, Xp] = particle_filter(t, M, A, B, C, X, R, Q, y, u)
    % sample
    for m = 1:M
        e = sqrt(R) * randn(1);
        Xp(m) = A * X(m) + B * u(t) + e;
        w(m) = normpdf(y(t), C * Xp(m), Q);
    end

    % re-sample
    W = cumsum(w);
    for m=1:M
        seed = W(end) * rand(1);
        X(m) = Xp(find(W > seed, 1));
    end

    muParticle = mean(X);
    SParticle = var(X);
end

function plot_first_time_step(t, Q, mup, mu, Sp, S, X, X0, Xp, y)
        L = 5;

        % Prior belief
        figure(1);
        clf;
        subplot(2,1,1); hold on;
        z = [(mu - L * sqrt(S)):(0.01):(mu + L * sqrt(S))];
        plot(z, normpdf(z, mu, S), 'b');
        plot([0 15], [1/15 1/15])
        title('Priors')
        axis([5 15 0 0.4])
        subplot(2, 1, 2);
        hold on;
        for m = 1:length(X0)
            plot([X0(m); X0(m)], [0 1], 'b')
        end
        axis([5 15 0 1])

        % Prediction step
        figure(2);
        clf;
        subplot(2,1,1);
        hold on;
        plot(z, normpdf(z, mu, S), 'b');
        z = [(mup - L * sqrt(Sp)):(0.01):(mup + L * sqrt(Sp))];
        plot(z, normpdf(z, mup, Sp),'r');
        title('Prior & Prediction')
        % legend('Prior', 'Prediction')
        axis([0 15 0 0.4])
        subplot(2, 1, 2);
        hold on;
        for m = 1:length(Xp)
            plot([Xp(m); Xp(m)], [0 1], 'r')
        end
        axis([0 15 0 1])

        % Measurement step
        figure(3);
        clf;
        subplot(2,1,1);
        hold on;
        plot(z, normpdf(z, mup, Sp), 'r');
        z = [(y(t) - L * sqrt(Q)):(0.01):(y(t) + L * sqrt(Q))];
        plot(z, normpdf(z, y(t), Q), 'g');
        z = [(mu - L * sqrt(S)):(0.01):(mu + L * sqrt(S))];
        plot(z,normpdf(z, mu, S), 'm');
        axis([0 15 0 0.6]);
        title('Prediction, Measurement & Belief')
        % legend('Prediction','Measurement', 'Belief')
        subplot(2,1,2); hold on;
        for m = 1:length(X)
            plot([X(m); X(m)], [0 1], 'm')
        end
        axis([0 15 0 1]);
end

%% main loop
tic;
for t=1:length(T)
    % select control action
    if (t>1)
        u(t) = u(t-1);
    end
    if (mu > 10)
        u(t) = 0;
    elseif (mu < 2);
        u(t) = 1;
    end

    % update state
    e = sqrt(R)*randn(1);
    x(t + 1) = A * x(t) + B * u(t) + e;

    % take measurement
    d = sqrt(Q) *randn(1);
    y(t) = C * x(t + 1) + d;

    % store prior
    % mu_old = mu;
    % S_old = S;

    % kalman filter estimation
    [mup, mu, Sp, S] = kalman_filter(t, A, B, C, R, Q, y, u, mu, S);

    % store kalman filter estimates
    mup_S(t) = mup;
    mu_S(t) = mu;

    % particle filter estimation
    [muParticle, SParticle, Xp] = particle_filter(t, M, A, B, C, X, R, Q, y, u);

    % store particle filter estimates
    muP_S(t) = muParticle;
    SP_S(t) = SParticle;

    % plot first time step results
    if (t == 1)
        plot_first_time_step(t, Q, mup, mu, Sp, S, X, X0, Xp, y);
    end
end
toc

%Plot full trajectory results
figure(4);
clf;
hold on;
plot(T, x(2:end),'b')
%plot(T,y,'rx')
%plot(T,u,'g');
plot(T, mu_S, 'r--')
plot(T, muP_S, 'co--')
plot(T, 2 * ones(size(T)), 'm--');
plot(T, 10 * ones(size(T)), 'm--');
title('State and estimates, M=10');
% legend('State', 'Kalman Estimate', 'Particle Estimate')
pause
