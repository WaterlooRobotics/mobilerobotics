% Example of error ellipse and Gaussian noise generation

% Define distribution
mu = [1; 2];
S = [4 -1; -1 1];

% Find eigenstuff
[SE, Se]=eig(S);

% Generate samples
samples = SE*sqrt(Se)*randn(2,10000);

% Create ellipse plots
figure(1);clf;hold on;
%error_ellipse(S,mu,0.5)
%error_ellipse(S,mu,0.99)
plot(mu(1) + samples(1,:),mu(2) + samples(2,:), 'r.')
axis equal