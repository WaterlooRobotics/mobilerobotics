% Kalman Filter Example 2: Multirate 2D Omnidirectional AUV example
% 'Estimation 1' topic slide 77
% State : position and velocity in north, east directions
% Motion : movement in north and east directions
% Measurement : measure position(10Hz) and velocity(100 Hz) in north and 
%               east directions
% Controller : Thrust in north and east directions 

clear all
clc
close all

% Example number
example = 3;

% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('multiratekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.01;

% Get motion model and measurement model
[A,B,R,numStates] = motion_model(example,dt);
[C,D,Q,m] = measurement_model(example);

% Prior
mu = zeros(numStates,1); % mean (mu)
S = 0.01*eye(numStates);% covariance (Sigma)

[QpE, Qpe] = eig(Q.Qp);
[QvE, Qve] = eig(Q.Qv);
[RE, Re] = eig (R);

% Store in a structure (State Space Model [ssm])
ssm.A = A;
ssm.B = B;
ssm.C = C;
ssm.D = D;
ssm.R = R;
ssm.Q = Q;
ssm.n = numStates;
ssm.m = m;

% Simulation Initializations
Tf = 1;
T = 0:dt:Tf;
u = 10*[sin(2*T);cos(T)];
numStates = length(A(1,:));
x = zeros(numStates,length(T));
x(:,1) = zeros(numStates,1);
mp = length(ssm.C.Cp(:,1));
mv = length(ssm.C.Cv(:,1));
y = zeros(mp,length(T));
mup_S = zeros(numStates,length(T));
mu_S = zeros(numStates,length(T));

% Multirate Kalman filter frequency (Hz) update for position measurement
freq = 10;

%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(numStates,1);
    % Update state
    x(:,t) = A*x(:,t-1)+ B*u(:,t) + e;

    % Take measurement
    % Select a measurement disturbance and determine measurement
    if (mod(t,10)==0)
        d = QpE*sqrt(Qpe)*randn(mp,1);
        y(:,t) = C.Cp*x(:,t) + d;
    else
        d = QvE*sqrt(Qve)*randn(mv,1);
        y([2 4],t) = C.Cv*x(:,t) + d;
    end
    
    %% Kalman Filter Estimation
    [mu,S,mup,Sp,K] = kalman_filter(ssm,mu,S,u(:,t),y(:,t),example,t,freq);
    
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