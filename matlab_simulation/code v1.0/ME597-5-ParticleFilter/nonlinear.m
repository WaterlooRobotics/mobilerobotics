%% Comparison of linear and nonlinear transformations

% Original Gaussian distribution
mu = 0; % mean (mu)
S = 1;% covariance (Sigma)
L = 5;
x = [mu-L*sqrt(S):0.01:mu+L*sqrt(S)]; % x points
px = normpdf(x,mu,S); % p(x) 
n = 5000000; % Number of samples to use
xS = sqrt(S)*randn(n,1); % Samples of original distribution

% Linear transformation
A = 2;
b = -2
yL = A*x + b;

% Resulting Gaussian distribution
muL = A*mu+b;
SL = A*S*A';
pyL = normpdf(yL,muL,SL);

% Nonlinear transformation
yN = atan(x+0.5); % Nonlinear transformation of pdf
yNS = atan(xS+0.5); % Nonlinear transformation of samples

% Resulting distribution
m = 100; % Number of bins to use
[d,YN] = hist(yNS,m); % Histogram of results
binw = (max(YN)-min(YN))/(m-1); % Bin width
pyN = d/n/binw; % Resulting distribution

% Gaussian approximation to resulting distribution
muN = mean(yNS); % Mean of samples of transformed distribution
SN = var(yNS); % Covariance of samples of transformed distribution
yNG = [muN-L*sqrt(SN):0.01:muN+L*sqrt(SN)];
pyNG = normpdf(yNG,muN,sqrt(SN));

% Linearized propagation of mean and covariance (a la EKF)
G = 1/((mu+1/2)^2+1);
muLN = atan(mu+0.5);
SLN = G*S*G';
yLN = [muLN-L*sqrt(SLN):0.01:muLN+L*sqrt(SLN)];
pyLN = normpdf(yLN,muLN,sqrt(SLN));

% Unscented approximation to resulting distribution
n = 1;
alpha = 2;
kappa = 1;
beta = 2;
lambda = alpha^2*(n+kappa)-n;

X(:,1) = mu;
Y(:,1) = atan(X(:,1)+0.5);
nlS = sqrt((n+lambda)*S);
for i = 1:n
   X(:,i+1) =  mu + nlS(:,i);
   Y(:,i+1) = atan(X(:,i+1)+0.5);
   X(:,n+i+1) =  mu - nlS(:,i);
   Y(:,n+i+1) = atan(X(:,n+i+1)+0.5);
end
wm(1) = lambda/(n+lambda);
wc(1) = wm(1) + (1-alpha^2+beta)+(1-alpha^2+beta);
wm(2:2*n+1) = 1/(2*(n+lambda));
wc(2:2*n+1) = 1/(2*(n+lambda));

muU = zeros(n,1);
SU = zeros(n,n);
for i = 1:2*n+1
    muU = muU+wm(i)*Y(:,i);
end
for i = 1:2*n+1
    SU = SU + wc(i)*(Y(:,i)-muU)*(Y(:,i)-muU)';
end
yNU = [muU-L*sqrt(SU):0.01:muU+L*sqrt(SU)];
pyNU = normpdf(yNU,muU,sqrt(SU));

%% Display results

L = 5; % number of std deviations to include

% Linear plot
figure(1);clf;
% Original distribution
subplot(2,2,4)
plot(x,px,'b');
title('Original')
xlabel('x')
ylabel('p(x)')
axis(1.2*[min(x) max(x) min(px) max(px)])
% Transformation function
subplot(2,2,2)
plot(x,yL,'g')
title('Linear Transformation y=Ax+b')
xlabel('x')
ylabel('y')
axis(1.2*[min(x) max(x) min(yL) max(yL)])
axis equal
% Resulting distribution
subplot(2,2,1)
plot(pyL,yL,'r');
title('Final')
xlabel('p(y)')
ylabel('y')
axis(1.2*[min(pyL) max(pyL) min(yL) max(yL)])

%Nonlinear plots
figure(2);clf;
subplot(2,2,4)
plot(x,px,'b');
title('Original')
xlabel('x')
ylabel('p(x)')
axis(1.2*[min(x) max(x) min(px) max(px)])
subplot(2,2,2)
plot(x,yN,'g')
title('Nonlinear Transformation y=tan^{-1}(x+1/2)')
xlabel('x')
ylabel('y')
axis(1.2*[min(x) max(x) min(yN) max(yN)])

subplot(2,2,1); hold on;
plot(pyN,YN,'r');
plot(pyNG,yNG,'m--');
plot([0 max(pyNG)], [muN muN], 'm--')
axis(1.2*[min(pyN) max(pyN) min(yN) max(yN)])
title('Resulting Distribution')
xlabel('p(y)')
ylabel('y')

figure(3); clf; hold on; % duplicate
plot(YN,pyN,'r', 'LineWidth', 1.5);
plot(yNG,pyNG,'m--', 'LineWidth', 1.5);
plot(yLN,pyLN,'g--','LineWidth', 1.5);
plot(yNU,pyNU,'c--','LineWidth', 1.5);
plot([muN muN],[0 max(pyNG)],'m--','LineWidth', 1.5)
plot([muLN muLN], [0 max(pyLN)],'g--','LineWidth', 1.5)
plot([muU muU], [0 max(pyNU)],'c--','LineWidth', 1.5)
plot(X,0.001*ones(1,length(X)), 'cx', 'MarkerSize', 8, 'LineWidth', 2);
plot(Y,0.001*ones(1,length(Y)), 'co', 'MarkerSize', 8, 'LineWidth', 2);
title('Resulting Distribution')
xlabel('y')
ylabel('p(y)')
legend('NL distribution', 'Best Gaussian Fit', 'EKF approx', 'UKF approx');
axis(1.2*[2*min(yN) 2*max(yN) min(pyN) max(pyN)])
