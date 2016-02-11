%% Least squares curve fit
% A demonstration of the least squares algorithm being used to identify a
% polynomial approximation to a nonlinear curve.  
% Note - can also use \ for unconstrained least squares, lsqlin for
% constrained linear least squares, lsqnonneg, lsqnonlin, lsqcurvefit.  See
% Matlab help for details, coded explicitly here to make connection to
% lecture clear.

clear; clc;

% Main function - f = 1/(1+25z_t^2) - sampled for plotting
z_t = -1:0.01:1; % range
N_t = length(z_t); % number of samples
b_t = 1./(1+25*z_t.^2); % function definition
figure(1); clf; hold on; 
plot(z_t,b_t) % Plot function values b_t wrt z_t

% Measurements taken at points along function
n = 10; % Number of measurements
e = 0.1^2*randn(1,n); % Gaussian noise added to measurements
z = [-1:2/(n-1):1]; % Distribute measurements along z axis
b = 1./(1+25*z.^2) + e; % Construct measurement values with additive noise
plot(z, b,'ro') % Plot measurement values b wrt z

% Polynomial approximation from measurement using least squares
m = 10; % Order of polynomial
x = ones(m,1); % Vector of polynomial coefficients to be determined with LS

% The function f_m = x(m)*z(i)^(m-1) + x(m-1)*z(i)^(m-2) + ... + x(1) is set
% equal to each measurement b(i), for the corresponding value of z(i).
% In form Ax = b, each measurement b(i) is equal to 
% A(i,:)*x = [ 1 z z^2 ... z^(m-1)]*x  = b(i)
for i=1:n % For each measurement
    A(i,1) = 1; % construct A(i,:) using z(i) through recursion
    for j=2:m
        aj = A(i,j-1)*z(i);
        A(i,j) = aj;
    end
end

% Define least squares solution
xe = inv(A'*A)*A'*b';
be = A*xe;
plot(z,be,'gx') % Plot polynomial approximation values be wrt measurement locations z

% Compare polynomial function to original function at all points in z_t
afit = ones(N_t,1);
for j = 2:m
    afit = [afit, afit(:,j-1).*z_t'];
end
bfit = afit*xe;
plot(z_t,bfit,'c')
ttl = sprintf('Least squares fit of %d measurements with polynomial of order %d', n, m);
title(ttl)
legend ('Function', 'Measurements', 'Approx. at Meas. Loc.', 'Approx. Function');
