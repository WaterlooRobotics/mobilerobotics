%% Linear program for robots
clear; clc;
% Linear dynamics
dt = 0.1;
A = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
B = [0 0; dt 0; 0 0; 0 dt];

% Input bounds
umax = 10;
umin = -10;

% Environment
% g = [ 0 1; 0 -1];
% b = [10; 0;];
g = [ 0 1; 0 -1; 1 0; -1 0];
b = [4.2; 0; 4.2; 0];

% Initial, final positions
p0 = [1 0 3 0 ];
pF = [4 0 1 0 0 0];

%% Optimization problem definition

% Time steps
T = 20;

% Number of optimization variables: n states and m inputs for T timesteps
n = length(A(1,:));
m = length(B(1,:));
N = (n+m)*T;

% Initial feasible solution
x0 = zeros(N,1);

% Cost
f = zeros(size(x0));
f(1:n+m:end) = -1;
f(3:n+m:end) = -1;
f(2:n+m:end) = 1;
f(4:n+m:end) = 1;
f(5:n+m:end) = 3;
f(6:n+m:end) = 3;

% Constraints
Aeq = zeros((n-1)*T,N);
beq = zeros((n-1)*T,1);
q = length(g(:,1));
Aineq = zeros(q*T,N);
bineq = zeros(q*T,1);
LB = -100*ones(N,1);
LB(5:6:end-1) = umin;
LB(6:6:end) = umin;
UB = 100*ones(N,1);
UB(5:6:end-1) = umax;
UB(6:6:end) = umax;

for i=1:T-1
    Aeq(n*(i-1)+1:n*i, (n+m)*(i)+1:(n+m)*(i)+4) = -eye(n);
    Aeq(n*(i-1)+1:n*i, (n+m)*(i-1)+1:(n+m)*(i-1)+4) = A;
    Aeq(n*(i-1)+1:n*i, (n+m)*(i-1)+5:(n+m)*(i-1)+6) = B;
    beq(n*(i-1)+1:n*i) = zeros(n,1);
end
Aeq(n*(T-1)+1:n*(T-1)+n,1:n) = eye(n);
Aeq(n*(T-1)+n+1:n*(T-1)+2*n+m,(n+m)*(T-1)+1:(n+m)*T) = eye(n+m);
% beq(n*(T-1)+1:n*(T-1)+n,1) = [p0'];
beq(n*(T-1)+1:n*(T-1)+2*n+m,1) = [p0'; pF'];

for i = 1:T
    Aineq(q*(i-1)+1:q*i, (n+m)*(i-1)+1:2:(n+m)*(i-1)+3) = g;
    bineq(q*(i-1)+1:q*i) = b;
end

figure(1); clf;spy(Aeq);
% Solve linear program
options = optimset('display', 'off');
tic;
[X,FVAL,EXITFLAG,OUTPUT,LAMBDA] = linprog(f,Aineq,bineq,Aeq,beq,LB,UB,[],options);
toc;
% Rename results
x = X(1:6:end);
vx = X(2:6:end);
y = X(3:6:end);
vy = X(4:6:end);
ax = X(5:6:end);
ay = X(6:6:end);

% Plot results
figure(2);clf; hold all;
plot(1:T,x)
plot(1:T,y)
plot(1:T,vx)
plot(1:T,vy)
plot(1:T,ax)
plot(1:T,ay)

figure(3); clf; hold on;
plot(x,y,'bo-');
plot(p0(1),p0(3),'bx');
plot(pF(1),pF(3),'ro');
plot(0,0,'md')
title('Linear Program path planning')
