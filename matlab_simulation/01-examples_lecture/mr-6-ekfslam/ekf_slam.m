% Extended Kalman Filter SLAM example
clear;clc;close all

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ekfSLAM.mp4');
    vidObj.Quality = 100;
    vidObj.FrameRate = 2;
    open(vidObj);
end


%% Select an example to run
% Example=1: robot travelling in a circular path between 2 lines of obstacles
% Example=2: robot travelling back and forth between 2 lines of obstacles
% Example=3: robot travelling in a circular path. 6 obstacles form a small circle
% Example=4: robot travelling in a circular path. 6 obstacles form a large circle
Example=1;

%% Main Code
% Time
Tf = 100;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State: (x,y,heading)
x0 = [0 0 0]';

% Generate control inputs and feature map based on example selection
switch Example
    case 1
        u = ones(2, length(T));
        u(2,:)=0.3*u(2,:);
        map = [-5:1:5 5:-1:-5; -2*ones(1,11) 8*ones(1,11)];
    case 2
        turns=5;    % number of 180 deg turns to make
        u=ones(2,length(T));
        u(2,:)=0;
        for n=1:turns
            u(2,floor(length(T)./(turns+1)*n))=pi./dt;
        end
        map=[-1:1:8,8:-1:-1; 2*ones(1,10),-2*ones(1,10)];
    case 3
        u = ones(2, length(T));
        u(2,:)=0.3*u(2,:);
        N=6; %number of features
        Radius=6;
        center=[1;3];
        theta=linspace(2*pi./N,2*pi,N);
        map=[center(1)+Radius.*cos(theta);
             center(2)+Radius.*sin(theta)];
    case 4
        u = ones(2, length(T));
        u(2,:)=0.3*u(2,:);
        N=6; %number of features
        Radius=12;
        center=[1;3];
        theta=linspace(2*pi./N,2*pi,N);
        map=[center(1)+Radius.*cos(theta);
             center(2)+Radius.*sin(theta)];
end
M = length(map);

% Motion Disturbance model
R = [0.001 0 0; 
     0 0.001 0; 
     0 0 0.001];
[RE, Re] = eig(R);

% Prior over robot state
mu0r = [0 0 0]'; % mean (mu)
S0rr = 0.00000000001*eye(3);% covariance (Sigma)

% Prior over feature map
S0mm=eye(2);  %predefined covariance for each feature when it is just detected;
newfeature = ones(M,1);

%Measurement model
rmax = 20; % Max range
thmax = pi/4; % 1/2 Field of view

% Measurement noise
Qi = [0.00001 0; 
     0 0.00001];
[QiE, Qie] = eig(Qi);


% Simulation Initializations
n = length(R(:,1)); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
N = n+2*M;
m = length(Qi(:,1)); % Number of measurements per feature 
y = zeros(m*M,length(T)); % Measurements

S=zeros(N,N);
S(1:n,1:n) = S0rr; % initially the map is empty, so the estimate and covariance only have the vehicle info

mu = zeros(N,1); % current belief
mu(1:n) = mu0r;

mu_S = zeros(N,length(T)); % Belief
mu_S(:,1) = mu;
count=0;    % count of detected features in the map

%% Plot initial step
t=1;
figure(1);clf; 
ekfSLAMplot(map,y,xr,mu_S,S,t,newfeature)
if (makemovie) 
    writeVideo(vidObj, getframe(gcf)); 
end
  
    
%% Main loop
tic;
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state
    xr(:,t) = simpleRobotMotionModel(xr(:,t-1),u(:,t),dt)+e;

    % Take measurements
    % For each feature
    flist = zeros(M,1);
    for i=1:M
        % If feature is visible
        if (inview(map(:,i)',xr(:,t),rmax,thmax))
            flist(i) = 1;

            fStart = (i-1)*2 + 1;
            fEnd = fStart + 1;
            
            % Select a motion disturbance
            d = QiE*sqrt(Qie)*randn(m,1);
            % Determine measurement
            y(fStart:fEnd,t) = range_bearing_meas_model(xr(:,t),map(:,i))+d;
        end
    end
    
    %% Extended Kalman Filter Estimation
    % Prediction update
    mu(1:3) = simpleRobotMotionModel(mu(1:3),u(:,t),dt);
    Gt = simpleLinearizedRobotMotionModel(mu,u(:,t),dt);
    S(1:n,1:n) = Gt*S(1:n,1:n)*Gt' + R;

    % Measurement update
    for i=1:M
        if flist(i) == 1
            fStart = n + (i-1)*2 + 1;
            fEnd = fStart + 1;

            % Feature initialization
            if newfeature(i) == 1
                count=count+1;

                % initialize the measurement
                mu(fStart:fEnd) = [ mu(1)+y(fStart-n,t)*cos(y(fEnd-n,t)+mu(3));
                                    mu(2)+y(fStart-n,t)*sin(y(fEnd-n,t)+mu(3)) ];
                % add the initial covariance estimate (I) for the new feature
                S(fStart:fEnd,fStart:fEnd) = S0mm;

                % don't initialize this feature again
                newfeature(i) = 0;

                disp([num2str(count) ' of the ' num2str(M) ' features have been detected'])
            end

            % Linearization
            % Predicted range
            Ht = range_bearing_meas_linearized_model(mu,i);

            % Measurement update
            K = S*Ht'* inv(Ht*S*Ht'+Qi);
            mu = mu + K*(y(fStart-n:fEnd-n,t) - range_bearing_meas_model(mu(1:3),mu(fStart:fEnd)));
            S = (eye(length(S))-K*Ht)*S;

            % In cases if S bemoes not positive definite, manually make it
            % P.D.
            if min(eig(S))<0
               S=S-eye(length(S)).*min(eig(S));
               warning('S was manually made positive definite')
            end

            % warn the user that linearization may not be accurate if the 
            % change in vehicle position is too large compared to input speed
            if norm(mu(1:2)-mu_S(1:2,t-1))>2*u(1,t)*dt
                warning('Linearization may have failed')
            end
        end
    end
 
    % Store results
    mu_S(1:length(mu),t) = mu;

    %% Plot results
    figure(1);clf; 
    ekfSLAMplot(map,y,xr,mu_S,S,t,newfeature)
    drawnow;
    if (makemovie) 
        writeVideo(vidObj, getframe(gcf)); 
    end
  
end

toc

if (makemovie) 
    close(vidObj); 
end
