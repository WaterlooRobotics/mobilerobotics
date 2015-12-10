%% Linear Quadratic Regulator - Pitch Control of Aircraft
clc; clear;
%% Simulation setup
t0 = 0;
tf = 100;
dt = 0.1;
T = t0:dt:tf;

%% Continuous time model

% State x = [angle of attack, pitch, pitch rate]
A=[-0.313 0 56.7; 0  0 56.7; -0.0139 0 -0.426];
B=[0.232; 0; 0.0203];
C=[0 1 0];
D=[0];

sys = ss(A,B,C,D);

%% Discretized model
sysd = c2d(sys, dt,'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

x0 = [1 0 0];

% LQR costs
Q = 0.01*eye(3);
R = 0.01*eye(1);

[x,u,Jx,Ju,P,K] = run_lqr(Ad,Bd,Q,R,t0,tf,dt,x0,0);
[xss,uss,Jxss,Juss,Pss,Kss] = run_lqr(Ad,Bd,Q,R,t0,tf,dt,x0,1);
[x2,u2,Jx2,Ju2,P2,K2] = run_lqr(Ad,Bd,Q,10*R,t0,tf,dt,x0,0);
[x3,u3,Jx3,Ju3,P3,K3] = run_lqr(Ad,Bd,Q,100*R,t0,tf,dt,x0,0);

% Default run plot
figure(1);hold on;clf;
subplot(4,1,1);hold on;
plot(T,x(1,:),'b-');
title('Angle of Attack')
subplot(4,1,2);hold on;
plot(T,x(2,:),'b-');
title('Pitch Angle')
subplot(4,1,3);hold on;
plot(T,x(3,:),'b-');
title('Pitch Rate')
subplot(4,1,4);hold on;
plot(T(1:end-1),u(:),'b-');
title('Deflection Angle')

% Plots for steady state
figure(2);hold on;clf;
subplot(4,1,1);hold on;
plot(T,x(1,:),'b-');
plot(T,xss(1,:),'r--');
title('Angle of Attack')
subplot(4,1,2);hold on;
plot(T,x(2,:),'b-');
plot(T,xss(2,:),'r--');
title('Pitch Angle')
subplot(4,1,3);hold on;
plot(T,x(3,:),'b-');
plot(T,xss(3,:),'r--');
title('Pitch Rate')
subplot(4,1,4);hold on;
plot(T(1:end-1),u(:),'b-');
plot(T(1:end-1),uss(:),'r-');
title('Deflection Angle')

figure(3);hold on;clf;
Pplot = reshape(P,length(Q)^2,length(P));
plot(T,Pplot([1:8],:)');
title('Costate')

% Plots for Q,R comparison
figure(4);hold on;clf;
subplot(4,1,1);hold on;
plot(T,x(1,:),'b-');
plot(T,x2(1,:),'r-');
plot(T,x3(1,:),'g-');
title('Angle of Attack')
subplot(4,1,2);hold on;
plot(T,x(2,:),'b-');
plot(T,x2(2,:),'r-');
plot(T,x3(2,:),'g-');
title('Pitch Angle')
subplot(4,1,3);hold on;
plot(T,x(3,:),'b-');
plot(T,x2(3,:),'r-');
plot(T,x3(3,:),'g-');
title('Pitch Rate')
subplot(4,1,4);hold on;
plot(T(1:end-1),u(:),'b-');
plot(T(1:end-1),u2(:),'r-');
plot(T(1:end-1),u3(:),'g-');
title('Deflection Angle')

figure(5);clf; hold on;
plot(Ju,Jx,'bx', 'MarkerSize',10,'LineWidth',2)
plot(Ju2,Jx2,'rx', 'MarkerSize',10,'LineWidth',2)
plot(Ju3,Jx3,'gx', 'MarkerSize',10,'LineWidth',2)
title('Cost Comparison, state vs control')
xlabel('Input costs');
ylabel('State error costs');
