% Multi-rate Kalman filter example
clear;clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('multiratekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.01;

% Prior
mu = zeros(4,1); % mean (mu)
S = 0.01*eye(4);% covariance (Sigma)

% Continuous motion model (2-D)
b = 1;
m = 2;
A = [ 0 1 0 0; 0 -b/m 0 0; 0 0 0 1; 0 0 0 -b/m];
B = [0 0 ;1/m 0; 0 0; 0 1/m];
R = [.0001 0 0 0; 0 .0001 0 0; 0 0 .0001 0; 0 0 0 .0001];
[RE, Re] = eig (R);

% Measurement model
Cp = eye(4);
Cv = zeros(2,4);
Cv(1,2) = 1;
Cv(2,4) = 1;
D = zeros(2,2);
Qp = [.004 0 -0.001 0 ; 0 0.1 0 -0.01;  -0.001 0 .001 0; 0 -0.01 0 0.05];
Qv = [.1 -0.01; -0.01 .05];
[QpE, Qpe] = eig (Qp);
[QvE, Qve] = eig (Qv);

% Form continuous system
sysc=ss(A,B,Cv,D);

% zoh discretization
sysd = c2d(sysc,dt,'zoh');
Ad = sysd.A;
Bd = sysd.B;

% Simulation Initializations
Tf = 1;
T = 0:dt:Tf;
u = 10*[sin(2*T);cos(T)];
n = length(Ad(1,:));
x = zeros(n,length(T));
x(:,1) = zeros(n,1);
mp = length(Cp(:,1));
mv = length(Cv(:,1));
y = zeros(mp,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));


%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = Ad*x(:,t-1)+ Bd*u(:,t) + e;

    % Take measurement
    % Select a measurement disturbance and determine measurement
    if (mod(t,10)==0)
        d = QpE*sqrt(Qpe)*randn(mp,1);
        y(:,t) = Cp*x(:,t) + d;
    else
        d = QvE*sqrt(Qve)*randn(mv,1);
        y([2 4],t) = Cv*x(:,t) + d;
    end
    
    %% Kalman Filter Estimation
    % Prediction update
    mup = Ad*mu + Bd*u(:,t);
    Sp = Ad*S*Ad' + R;

    % Measurement update
    if (mod(t,10) == 0)
        K = Sp*Cp'*inv(Cp*Sp*Cp'+Qp);
        mu = mup + K*(y(:,t)-Cp*mup);
        S = (eye(n)-K*Cp)*Sp;
    else
        K = Sp*Cv'*inv(Cv*Sp*Cv'+Qv);
        mu = mup + K*(y([2 4],t)-Cv*mup);
        S = (eye(n)-K*Cv)*Sp;
    end
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = [K(:,1); K(:,2)];
  
    %% Plot results
    figure(1);clf; hold on;
    plot(x(3,2:t),x(1,2:t), 'ro--')
    if (mod(t,10)==0) plot(y(3,t),y(1,t), 'gx'); end
    %plot(mup_S(3,1:t),mup_S(1,1:t), 'mx--')
    plot(mu_S(3,2:t),mu_S(1,2:t), 'bx--')
    mu_pos = [mu(3) mu(1)];
    S_pos = [S(3,3) S(3,1); S(1,3) S(1,1)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and beliefs')
    legend('State', 'Measurement','Estimate')
    axis([-.5 2.5 -.5 2])
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    
end
if (makemovie) close(vidObj); end


