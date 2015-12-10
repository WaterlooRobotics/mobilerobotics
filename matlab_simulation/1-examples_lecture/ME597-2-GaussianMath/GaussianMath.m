%% Gaussian Math
% Test addition, multiplication and division of Gaussians to see if result
% is still Gaussian, by sampling excessively and performing computations on
% samples, then comparing histograms to Gaussian approximations of final 
% distribution.

% Distribution samples
n = 100000;
x = randn(n,1);
y = 2*randn(n,1);

% Math
z1 = x+y;
z2 = x.*y;
z3 = x./y;

% Histogram fit

figure(1); clf; hold on;
histfit(z1);
xlim([-10 10])

figure(2); clf; hold on;
histfit(z2);
xlim([-10 10])

figure(3); clf; hold on;
histfit(z3,10000);
xlim([-200 200])
