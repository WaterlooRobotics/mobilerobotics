% Continuous to discrete dynamics example
% Compares the discretization approaches described in lecture for a linear
% state space model of an ROV with drag

% Discrete time step
dt = 0.1; % coarse
dtc = 0.0001; % fine

% Continuous model (2-D)
b = 1;
m = 2;
A = [ 0 1 0 0; 0 -b/m 0 0; 0 0 0 1; 0 0 0 -b/m];
B = [0 0 ;1/m 0; 0 0; 0 1/m];
C = eye(4);
D = zeros(4,2);
sysc=ss(A,B,C,D);

% Discrete model (rough approximation)
Ad = [ 1 dt 0 0; 0 1-b/m*dt 0 0; 0 0 1 dt; 0 0 0 1-b/m*dt];
Bd = [0 0 ;1/m*dt 0; 0 0; 0 1/m*dt];
Cd = eye(4);
Dd = zeros(4,2);
sysd1 = ss(Ad,Bd,Cd,Dd,dt);

% True zoh discretization
sysd2 = c2d(sysc,dt,'zoh');

% foh discretization
sysd3 = c2d(sysc,dt,'foh');

% Tustins discretization
sysd4 = c2d(sysc,dt,'tustin');

% Simulation Initializations
t = 0:dt:10;  % Coarse time vector
u = [sin(t);cos(t)];
x0 = [0 0 0 0]';

tc = 0:dtc:10; % Fine time vector
uc = [sin(tc);cos(tc)];


% Simulations - performed using lsim
yc=lsim(sysc,uc,tc,x0);
y1=lsim(sysd1,u,t,x0);
y2=lsim(sysd2,u,t,x0);
y3=lsim(sysd3,u,t,x0);
y4=lsim(sysd4,u,t,x0);

% Plot trajectoies
figure(1);clf; hold on;
plot(yc(:,3),yc(:,1),'b')
plot(y1(:,3),y1(:,1),'r')
plot(y2(:,3),y2(:,1),'g')
plot(y3(:,3),y3(:,1),'m')
plot(y4(:,3),y4(:,1),'c')
axis equal
legend('Continuous','Rough','zoh','foh','tustin');
xlabel('East')
ylabel('North')
title('State propagation using various discretizations')


