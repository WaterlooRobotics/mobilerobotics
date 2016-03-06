function [w_m,w_c,lambda] = WeightedValues(kappa,alpha,beta,n)
%Calculates weights on the samples based on input parameters
    lambda = alpha^2*(n+kappa)-n;
    w_m(1) = lambda/(n+lambda);
    w_c(1) = lambda/(n+lambda) + (1-alpha^2+beta);
    w_m(2:2*n+1) = 1/(2*(n+lambda));
    w_c(2:2*n+1) = 1/(2*(n+lambda));

end
