% Extended Kalman Filter SLAM example
clear;clc;close all

%% Create AVI object
makemovie = 1;
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
Tf = 40;
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
        end;
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

mu = mu0r;
S=S0rr; % initially the map is empty, so the estimate and covariance only have the vehicle info

mu_S = zeros(N,length(T)); % Belief
mu_S(1:3,1) = mu;
count=0;    % count of detected features in the map

%% Plot initial step
t=1;
figure(1);clf; 
ekfSLAMplot(map,y,xr,mu_S,S,t,newfeature)
if (makemovie) writeVideo(vidObj, getframe(gcf)); end
  
    
%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state
      xr(:,t) = two_wheel_motion_model(xr(:,t-1),u(:,t),dt)+e;

    % Take measurements
    % For each feature
    flist = zeros(M,1);
    for i=1:M
        % If feature is visible
        if (inview(map(:,i),xr(:,t),rmax,thmax))
            flist(i) = 1;
            % Select a motion disturbance
            d = QiE*sqrt(Qie)*randn(m,1);
            % Determine measurement
            y(2*(i-1)+1:2*i,t) = range_bearing_meas_model(xr(:,t),map(:,i))+d;
        end
    end
    
    %% Extended Kalman Filter Estimation
    % Prediction update
    mu(1:3)= two_wheel_motion_model(mu(1:3),u(:,t),dt);
    predicted_xr=mu(1:3);
    Gt = two_wheel_motion_linearized_model(mu,u(:,t),dt);
    S(1:n,1:n) = Gt*S(1:n,1:n)*Gt' + R;

    % Measurement update
    for i=1:M
        if (flist(i))
         % j is the index for the measured feature. It is needed because
         % when the map is updated and rearranged, j will be diff from i
            j=i;
            % Feature initialization
            if (newfeature(i) == 1)
                count=count+1;
                % rearrange the order of estimates so that those of the newly
                % detected feture is placed on top, right after the vehicle
                % states
                if count==1
                   mu=[mu(1:3);
                        mu(1)+y(2*(i-1)+1,t)*cos(y(2*i,t)+mu(3));
                        mu(2)+y(2*(i-1)+1,t)*sin(y(2*i,t)+mu(3))];
                else
                    mu=[mu(1:3);
                        mu(1)+y(2*(i-1)+1,t)*cos(y(2*i,t)+mu(3));
                        mu(2)+y(2*(i-1)+1,t)*sin(y(2*i,t)+mu(3));
                        mu(4:end)];  % augment measured features into states
                end
                % rearrange the covariance matrix. Those of the newly
                % discovered features are set to be S0mm, a predefined
                % value. Off-diagonal terms are set to 0.
                temp=zeros(length(S)+2,length(S)+2);
                temp(1:3,1:3)=S(1:3,1:3);
                temp(4:5,4:5)=S0mm;
                if length(S)>3
                    temp(6:end,1:3)=S(4:end,1:3);
                    temp(1:3,6:end)=S(1:3,4:end);
                    temp(6:end,6:end)=S(4:end,4:end);
                end
                S=temp;
                newfeature(i) = 0;
                % rearrange map, newfeature, y, and flist so that they
                % follow the same order as mu
                [map, newfeature, y, flist]=rearrangeMap(map,newfeature,y,flist,i,count);
                % after rearranging, this newly discovered feature is the
                % first feature in the list
                j=1;
                disp([num2str(count) ' of the ' num2str(M) ' features have been detected'])
            end
           
            % Linearization
            % Predicted range
            dx = mu(3+2*(j-1)+1)-mu(1);
            dy = mu(3+2*j)-mu(2);
            rp = sqrt((dx)^2+(dy)^2);
            Ht = range_bearing_meas_linearized_model(mu,j);
            I = y(2*(j-1)+1:2*j,t)- range_bearing_meas_model(mu(1:3),mu((3+2*(j-1)+1):3+2*j));
 
            % Measurement update
            K = S*Ht'*inv(Ht*S*Ht'+Qi);
            mu = mu + K*I;
            S = (eye(length(S))-K*Ht)*S;
            
            % In cases if S bemoes not positive definite, manually make it
            % P.D.
            if min(eig(S))<0
               S=S-eye(length(S)).*min(eig(S));
               warning('S was manually made positive definite')
            end
            
            % warn the user that linearization may not be accurate if the 
            % change in vehicle position is too large compared to input speed
            if norm(mu(1:2)-mu_S(1:2,t-1))>2.*u(1,t).*dt
                warning('Linearization may have failed')
            end
        end
    end
 
    % Store results
    mu_S(1:length(mu),t) = mu;

    %% Plot results
    figure(1);clf; 
    ekfSLAMplot(map,y,xr,mu_S,S,t,newfeature)
   
    if (makemovie) writeVideo(vidObj, getframe(gcf)); end
  
end
if (makemovie) close(vidObj); end