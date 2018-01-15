%% Random Noise simulation
% Generates Gaussian samples with specified mean and covariance, and
% places error ellipses of different probabilities on top.

clear; clc;

mu = [0; 0];
Q = [1 -1; -1 2]; % Covariance
n = length(Q(:,1)); % Size
[QE, Qe] = eig(Q) % Eigenvectors and eigenvalues of Sigma

% Create a second Gaussian that swaps eigenvalues (or eigenvectors)
RE = QE;
Re(1,1) = Qe(2,2);
Re(2,2) = Qe(1,1);

% Create sample sets to represent the Gaussian distributions
S =  10000;
for i = 1:S
    ra(:,i) = randn(n,1);  % Generate normal samples
    q(:,i) = mu + QE*sqrt(Qe)*ra(:,i); % Convert to samples with mean mu and covariance Q
    r(:,i) = mu + RE*sqrt(Re)*ra(:,i); % mean mu and covariance R
end

% Plot original distribution and error ellipses
figure(1); clf; hold on;
plot(q(1,:), q(2,:),'g.')
error_ellipse(Q,mu,0.75);
error_ellipse(Q,mu,0.95);
axis equal

% Plot modified distribution and original error ellipses
figure(2); clf; hold on;
plot(r(1,:), r(2,:),'g.')
mu = [0 0];
error_ellipse(Q,mu,0.75);
error_ellipse(Q,mu,0.95);
axis equal


