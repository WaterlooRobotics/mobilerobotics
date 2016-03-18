% Extended Kalman Filter SLAM example
clear;clc; close all;

% Time
Tf = 60;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State
x0 = [0 0 0]';

% Control inputs
u = ones(2, length(T));
u(2,:)=0.3*u(2,:);

% Motion Disturbance model
R = [0.001 0 0; 
     0 0.001 0; 
     0 0 0.001];
% R=zeros(3,3);
[RE, Re] = eig(R);

% Prior over robot state
mu0r = [0 0 0]'; % mean (mu)
S0rr = 0.00000000001*eye(3);% covariance (Sigma)

% Feature Map
N=6; %number of features
Radius=12;
center=[1;3];
theta=linspace(2*pi./N,2*pi,N);
map=[center(1)+Radius.*cos(theta);
     center(2)+Radius.*sin(theta)];
M = length(map);

% Prior over feature map
S0mm=0.1*eye(2);  %%% predefined covariance for each feature when it is just detected;
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
S=S0rr;

mu_S = zeros(N,length(T)); % Belief
mu_S(1:3,1) = mu;
count=0;    %%%%%%%%%% count of detected features;

%% Plot results
t=1;
    figure(1);clf; 
    subplot(1,2,1); hold on;
    plot(map(1,:),map(2,:),'go', 'MarkerSize',10,'LineWidth',2);
    plot(xr(1,1:t),xr(2,1:t), 'ro--')
    plot([xr(1,t) xr(1,t)+1*cos(xr(3,t))],[xr(2,t) xr(2,t)+1*sin(xr(3,t))], 'r-')
    plot(mu_S(1,1:t),mu_S(2,1:t), 'bx--')
    plot([mu_S(1,t) mu_S(1,t)+1*cos(mu_S(3,t))],[mu_S(2,t) mu_S(2,t)+1*sin(mu_S(3,t))], 'b-')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    axis equal
    title('SLAM with Range & Bearing Measurements')
    subplot(1,2,2);
    image(10000*S);
    colormap('gray');
    axis([0,N,0,N])
    axis('square')%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    title('Covariance matrix')
    F(t) = getframe(gcf);

    
%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state
      xr(:,t) = two_wheel_motion_model(xr(:,t-1),u(:,t),dt)+e;  %%%%%%%%

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
              y(2*(i-1)+1:2*i,t) = range_bearing_meas_model(xr(:,t),map(:,i))+d;%%%%%%%%%%%
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
            % when the map is updated and rearranged j will be diff from i
            j=i;
            % Feature initialization
            if (newfeature(i) == 1)
                count=count+1;
                % rearrange the estimate so that those of the newly
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
                % value. Off-diagonal terms are set to be 0.
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
                j=1;
                disp([num2str(count) ' of the ' num2str(M) ' features have been detected'])
            end
            
            % Linearization
            % Predicted range
            dx = mu(3+2*(j-1)+1)-mu(1);
            dy = mu(3+2*j)-mu(2);
            rp = sqrt((dx)^2+(dy)^2);
            
            Ht = range_bearing_meas_linearized_model(mu,j);%%%%%%%%%%%%%
            I = y(2*(j-1)+1:2*j,t)- range_bearing_meas_model(mu(1:3),mu((3+2*(j-1)+1):3+2*j));%%%%%%%%%%%%5

            % Measurement update
            K = S*Ht'*inv(Ht*S*Ht'+Qi);
            mu = mu + K*I;
            S = (eye(length(S))-K*Ht)*S;
            
            if norm(mu(1:2)-mu_S(1:2,t-1))>2.*u(1,t).*dt
                disp('Something went wrong!')
                warning('Linearization may have failed')
%                 disp(['Now dy=' num2str(dy) ' and dx=' num2str(dx)])
            end
        end
    end
 
    % Store results.
    mu_S(1:length(mu),t) = mu;


    %% Plot results
    figure(1);clf; 
    subplot(1,2,1); hold on;
    plot(map(1,:),map(2,:),'go', 'MarkerSize',10,'LineWidth',2);
    plot(xr(1,1:t),xr(2,1:t), 'ro--')
    plot([xr(1,t) xr(1,t)+1*cos(xr(3,t))],[xr(2,t) xr(2,t)+1*sin(xr(3,t))], 'r-')
    plot(mu_S(1,1:t),mu_S(2,1:t), 'bx--')
    plot([mu_S(1,t) mu_S(1,t)+1*cos(mu_S(3,t))],[mu_S(2,t) mu_S(2,t)+1*sin(mu_S(3,t))], 'b-')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);

    for i=1:M
          if (~newfeature(i))
              fi = 2*(i-1)+1;
              fj = 2*i;
              plot([xr(1,t) xr(1,t)+y(fi,t)*cos(y(fj,t)+xr(3,t))], [xr(2,t) xr(2,t)+y(fi,t)*sin(y(fj,t)+xr(3,t))], 'c');
              plot(mu(3+fi),mu(3+fj), 'gx')
              mu_pos = [mu(3+fi) mu(3+fj)];
              S_pos = [S(3+fi,3+fi) S(3+fi,3+fj); S(3+fj,3+fi) S(3+fj,3+fj)];
              error_ellipse(S_pos,mu_pos,0.75);
          end
    end
    axis equal
%     axis([-4 6 -1 7])
    title('SLAM with Range & Bearing Measurements')
    
    subplot(1,2,2);
    image(10000*S);
    colormap('gray');
    axis('square')%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    axis([0,N,0,N])%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    title('Covariance matrix')
 
    F(t) = getframe(gcf);
    
end

