%% Flyover Localization
clear;clc;

% Time
Tf = 20;
dt = 0.1;
T = 0:dt:Tf;


% Initial State
x0 = [0 0 0 -0.5 2]';

% Feature Map
map = [ -10 0;  
         10 0;
         10 20;
         -10 20];

% Control inputs
u = ones(2, length(T));
u(1,:)=5*u(2,:);
u(2,:)=0.3*u(2,:);

% Disturbance model
R = [0.01 0 0 0 0; 
     0 0.01 0 0 0; 
     0 0 0.0001 0 0;
     0 0 0 0.001 0;
     0 0 0 0 0.001];
 
[RE, Re] = eig(R);

% Measurement type and noise
Q = 0.01;
[QE, Qe] = eig(Q);

% Number of particles
D = 100;

% Prior - uniform over position, heading and wind ranges
r_pos = 5;
r_head = pi/6;
r_wind = 2;

X = zeros(5,D);
X(1,:) = x0(1) + r_pos*rand(1,D) - r_pos/2;
X(2,:) = x0(2) + r_pos*rand(1,D) - r_pos/2;
X(3,:) = x0(3) + r_head*rand(1,D) - r_head/2;
X(4,:) = x0(4) + r_wind*rand(1,D) - r_wind/2;
X(5,:) = x0(5) + r_wind*rand(1,D) - r_wind/2;

% Particle filter initialization
X0 = X;
Xp = X;
w = zeros(1,D);

% Simulation Initializations
n = length(x0);
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1))*length(map);
y = zeros(m,length(T));
Qm = Q*eye(length(map));

figure(1);clf; hold on;
plot(map(:,1),map(:,2),'go', 'MarkerSize',10,'LineWidth',2);
plot(x(1,1),x(2,1), 'ro--')
for dd=1:D
    plot(X(1,dd),X(2,dd),'b.')
end
axis equal
axis([-4 6 -1 7]);
title('Particle Filter Localization')


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = [x(1,t-1)+(u(1,t)*cos(x(3,t-1))+x(4,t-1))*dt;
              x(2,t-1)+(u(1,t)*sin(x(3,t-1))+x(5,t-1))*dt;
              x(3,t-1)+u(2,t)*dt;
              x(4,t-1);
              x(5,t-1)] + e;
    
    % Select a motion disturbance
    d = QE*sqrt(Qe)*randn(4,1);
    % Determine measurement
    y(:,t) = [atan2(map(:,2)-x(2,t),map(:,1)-x(1,t))-x(3,t)] + d;

    %% Particle filter estimation
    for dd=1:D
        e = RE*sqrt(Re)*randn(n,1);
        Xp(:,dd) = [X(1,dd)+(u(1,t)*cos(X(3,dd))+X(4,dd))*dt;
                    X(2,dd)+(u(1,t)*sin(X(3,dd))+X(5,dd))*dt;
                    X(3,dd)+u(2,t)*dt;
                    X(4,dd);
                    X(5,dd)] + e;
        hXp = [atan2(map(:,2)-Xp(2,dd),map(:,1)-Xp(1,dd))-Xp(3,dd)] + d;
        w(dd) = mvnpdf(y(:,t),hXp,Qm);
    end
    W = cumsum(w);
    for dd=1:D
        seed = max(W)*rand(1);
        X(:,dd) = Xp(:,find(W>seed,1));
    end
    muParticle = mean(X');
    SParticle = var(X');

    muParticle_S(:,t) = muParticle;
    
     %% Plot results
     if (mod(t,10)==0)
        plot(x(1,1:t),x(2,1:t), 'ro--')
        for kk=1:length(map)
            plot([x(1,t) x(1,t) + 30*cos(y(kk,t)+x(3,t))], [ x(2,t) x(2,t) + 30*sin(y(kk,t)+x(3,t))], 'c')
        end
        for dd=1:D
            plot(X(1,dd),X(2,dd),'b.')
        end
     end
        axis equal
     title('Particle Filter Localization')
     drawnow;
end

figure(2);clf;hold on;
plot(T, x(4,:),'b-');
plot(T,muParticle_S(4,:),'b--')
plot(T, x(5,:),'r-');
plot(T,muParticle_S(5,:),'r--')
title('Wind state estimates')
legend('Wind-X','Wind-X_{est}','Wind-Y','Wind-Y_{est}')

