function [x,y,u,mup_S,mu_S] = initialization(example,T_len,n,m,S)

% This function performs initialization given a certain example

if example == 1
    x = zeros(1,T_len);
    x(1) = mu+sqrt(S)*randn(1);
    y = zeros(1,T_len);
    u = y;
    mup_S = zeros(n,T_len);
    mu_S = zeros(n,T_len);
end

if example == 2
    x = zeros(n,T_len);
    x(:,1) = zeros(n,1);
    y = zeros(m,T_len);
    mup_S = zeros(n,T_len);
    mu_S = zeros(n,T_len);
    u = zeros(2,T_len);
end

if example == 3
    x = zeros(n,T_len);
    x(:,1) = zeros(n,1);
    y = zeros(m.mp,T_len);
    mup_S = zeros(n,T_len);
    mu_S = zeros(n,T_len);
    u = zeros(2,T_len);
end