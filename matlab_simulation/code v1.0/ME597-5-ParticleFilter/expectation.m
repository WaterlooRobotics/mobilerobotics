%Expectation 
clear; clc;
% Distribution
L=3;
mu1 = 0; % mean (mu)
S1 = .3;% std deviation
x = [mu1-L*sqrt(S1):0.01:mu1+L*sqrt(S1)];
xmin = min(x); xmax = max(x);
fx = normpdf(x,mu1,S1); % p(x) 
Fx = cumsum(fx);

A = [-0.5 0.2];

I = zeros(1,length(x))
ind1 = find(x>A(1),1)
ind2 = find(x<A(2),1, 'last')
I(ind1:ind2) = 1

figure(1);clf;hold on;
plot(x,fx, 'LineWidth',2);
plot(x,I,'g','LineWidth',2);
plot(x,fx.*I,'r','LineWidth',2);
title('E_f[I(x \in A)]')
legend('f(x)', 'I(x \in A)', 'f(x)I(x \in A)');