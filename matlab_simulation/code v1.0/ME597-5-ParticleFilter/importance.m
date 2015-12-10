%% Importance Sampling graphic
clear;clc;
%% Generate two interesting distributions

% First distribution
L=5;
mu = 0; % mean (mu)
S = 0.5;% covariance (Sigma)
x = [mu-L*sqrt(S):0.005:mu+L*sqrt(S)]; % x points
gx = normpdf(x,mu,S); % p(x) 
gx = gx + 0.1; % Add uniform base probability
gx = gx/sum(gx); % Normalize 
Gx = cumsum(gx);

% Second distribution
mu1 = -1;
S1 = .25;
fx1 = normpdf(x,mu1,S1);
mu2 = -0;
S2 = 0.5;
fx2 = normpdf(x,mu2,S2);
fx = fx1+fx2 + 0.1; % Form f(x)
fx = fx/sum(fx); % Normalize 
Fx = cumsum(fx);

%% Importance Sampling

% Number of particles
M = 20000;

% Sampling g(x)
draw = rand(1,M);
for m=1:M
    indP(m) = find(Gx>=draw(m),1);
    xP(m) = x(indP(m));
end
xPind = hist(indP,M);


% Weighting f(x)/g(x)
for m=1:M
    pgx(m) = gx(find(x>=xP(m),1));
    pfx(m) = fx(find(x>=xP(m),1));
    wx(m) = pfx(m)/pgx(m);
end

%Resampling
WX = cumsum(wx);
WX = WX/max(WX);

draw = rand(1,M);
for m=1:M
    indPnew(m) = find(WX>=draw(m),1);
    xPnew(m) = xP(indPnew(m));
end
xPindnew = hist(indPnew,M);

%% Plotting

% Plot of both distributions, and initial sample of g(x)
figure(1);clf; 
% f(x) and g(x)
subplot(3,1,1); hold on;
plot(x,fx/0.01,'b');
plot(x,gx/0.01,'r');
title('Distributions, and Samples from g(x)')
legend('f(x)', 'g(x)')
axis([-3 2 0 0.7])
% Actual samples plotted as bars, solidity represents density
subplot(3,1,2); hold on;
for m=1:25:M
    plot([xP(m);xP(m)],[0 1],'r')
end
ylabel('Samples');
axis([-3 2 0 1])
% Histogram of samples into 200 bins
subplot(3,1,3); hold on;
[xPH,xH] = hist(xP,200);
plot(xH,xPH./M,'r');
xlabel('x');
ylabel('# samples');
axis([-3 2 0 1.1*max(xPH./M)])

% Weights for each sample
figure(2);clf;
subplot(2,1,1); hold on;
plot(x,fx/0.001,'b');
plot(x,gx/0.001,'r');
plot(x,fx./gx,'g');
title('Weights for each sample of g(x)')
legend('f(x)', 'g(x)','f(x)/g(x)')
axis([-3 2 0 7])
subplot(2,1,2); hold on;
for m=1:25:M
    plot([xP(m);xP(m)],[0 wx(m)],'g')
end
xlabel('x')
ylabel('Weights f(x)/g(x)')
axis([-3 2 0 max(wx)])

figure(3);clf;
subplot(2,1,1); hold on;
plot(x,fx/0.01,'b');
plot(x,gx/0.01,'r');
title('Distributions, and importance sampled f(x)')
legend('f(x)', 'g(x)')
axis([-3 2 0 0.7])

subplot(2,1,2); hold on;
[xPnewH,xnewH] = hist(xPnew,200);
plot(xnewH,xPnewH./M,'b');
xlabel('x');
ylabel('# samples');
axis([-3 2 0 1.1*max(xPnewH./M)])
