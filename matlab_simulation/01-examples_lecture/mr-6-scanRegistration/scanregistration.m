%% Scan Registration example
clear;clc;

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ICPmap2.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 5;
    open(vidObj);
end

% Time
Tf = 100;
dt = 0.5;
T = 0:dt:Tf;

% True map
M = 100;
N = 100;
I = imread('funnymap.jpg');
I = imresize(I, [M N]);
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map); % Convert to 0 free, 1 occupied and flip.

% Map and vehicle path
figure(1);clf; hold on;
image(100*(1-map));
colormap(gray);
axis equal
axis([0 N 0 M]);

% Initial Robot State
x0 = [50 5 0]';

% Motion Disturbance model
R = [0.001 0 0; 
     0 0.001 0; 
     0 0 0.0001];
[RE, Re] = eig(R);

% Control inputs
u = ones(2, length(T));
u(1,:)=3*u(2,:);
u(2,:)=0.07*u(2,:);

% Measurement model
rmax = 30;
meas_phi = [-pi/2:0.05:pi/2];
Q = 0.0002;
d = sqrt(Q)*randn(1,1);
meas_r = getranges(map,x0,meas_phi,rmax) ;

% Make current measurement point cloud
kk=1;
for jj = 1:length(meas_phi)
    if(meas_r(jj) < rmax)
        pc(jj,:) = [x0(1)+meas_r(jj)*cos(meas_phi(jj)+x0(3)), x0(2)+meas_r(jj)*sin(meas_phi(jj)+x0(3)), 0];
        kk = kk+1;
    end
end

% Simulation Initializations
n = length(R(:,1)); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
xICP = zeros(n, length(T));
diffT = zeros(n, length(T));
diffICP = zeros(n,length(T));
errICP = zeros(1, length(T));
mur = x0;

% Belief map
m = 0.5*ones(M,N);
L0 = log(m./(1-m));
L=L0;

  
%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state
    xr(:,t) = [xr(1,t-1)+u(1,t)*cos(xr(3,t-1))*dt;
              xr(2,t-1)+u(1,t)*sin(xr(3,t-1))*dt;
              xr(3,t-1)+u(2,t)*dt] + e;

    xc = max(1,min(N,round(xr(1,t))));
    yc = max(1,min(M,round(xr(2,t))));

    % Predicted robot state
    mur(:,t) = [mur(1,t-1)+u(1,t)*cos(mur(3,t-1))*dt;
              mur(2,t-1)+u(1,t)*sin(mur(3,t-1))*dt;
              mur(3,t-1)+u(2,t)*dt];

    % Quantum tunneling through obstacles
    if (map(xc,yc)) 
        continue; 
    end
          
    % Define measurements
    d = sqrt(Q)*randn(1,1);
    meas_r = getranges(map,xr(:,t),meas_phi,rmax);
    
    %% ICP
    % Store previous point cloud
    pc_prev = pc;

    % Make current measurement point cloud
    kk = 1; pc = [];
    for jj = 1:length(meas_phi)
        if (meas_r(jj) < rmax)
            pc(kk,:) = [xr(1,t)+meas_r(jj)*cos(meas_phi(jj)+xr(3,t)), xr(2,t)+meas_r(jj)*sin(meas_phi(jj)+xr(3,t)), 0]; 
            kk=kk+1;
        end
    end

    % Perform ICP scan registration
    [Ricp, Ticp, ER, tau] = icp(pc_prev', pc',10);
    % Transform pointcloud for display
    pc_new = zeros(size(pc));
    for kk = 1:length(pc)
        pc_new(kk,:) = (Ricp*pc(kk,:)' + Ticp)'; 
    end
    
    % Store robot pose estimate
    xICP(:,t) = Ricp*(mur(:,t)) + Ticp;

    % Plot scan registration results
    figure(5); clf;hold on;
    plot(pc(:,2), pc(:,1),'bo');
    plot(pc_prev(:,2), pc_prev(:,1),'ro');
    plot(pc_new(:,2), pc_new(:,1),'go');
    axis equal
    diffT(:,t) = (xr(:,t)-xr(:,t-1));
    diffICP(:,t) = Ticp;
    errICP(t) = norm(diffT(:,t)-diffICP(:,t));
    
   
    %% Map update
    measL = zeros(M,N);
    for i = 1:length(meas_phi)
        % Get inverse measurement model (with scan registration estimate or
        % motion prediction)
        invmod = inversescannerbres(M,N,xICP(1,t),xICP(2,t),meas_phi(i)+xICP(3,t),meas_r(i),rmax);
        % invmod = inversescannerbres(M,N,mur(1,t),mur(2,t),meas_phi(i)+mur(3,t),meas_r(i),rmax);
        for j = 1:length(invmod(:,1))
            ix = invmod(j,1);
            iy = invmod(j,2);
            il = invmod(j,3);
            % Calculate updated log odds
            L(ix,iy) = L(ix,iy) +log(il./(1-il))-L0(ix,iy);
            measL(ix,iy)= measL(ix,iy) +log(il./(1-il))-L0(ix,iy);
        end
    end
    % Calculate probabilities
    m = exp(L)./(1+exp(L));
    invmod_T = exp(measL)./(1+exp(measL));

    % Map, scans and vehicle path
    figure(1);clf; hold on;
    image(100*(1-map));
    colormap(gray);
    plot(xr(2,1:t),xr(1,1:t),'bx-')
    plot(xICP(2,2:t),xICP(1,2:t),'go')
    for jj = 1:length(pc_new(:,1))
        plot(pc_new(jj,2),pc_new(jj,1),'rx'); 
    end
    for jj = 1:length(pc_prev(:,1))
            plot(pc_prev(jj,2),pc_prev(jj,1),'bo'); 
    end
    axis([0 N 0 M])

    % Inverse measurement model
    figure(2);clf; hold on;
    image(100*(invmod_T));
    colormap(gray);
    plot(xr(2,t),xr(1,t),'bx')
    for i=1:length(meas_r)
        plot(xr(2,t)+meas_r(i)*sin(meas_phi(i) + xr(3,t)),xr(1,t)+meas_r(i)*cos(meas_phi(i)+ xr(3,t)),'ko')
    end
    axis([0 N 0 M])

    % Belief map
    figure(3);clf; hold on;
    image(100*(1-m));
    colormap(gray);
    plot(mur(2,max(1,t-10):t),mur(1,max(1,t-10):t),'bx-')
    axis([0 N 0 M])
    %F3(t-1) = getframe;
    title('Current occupancy grid map')
    
    if (makemovie) 
        writeVideo(vidObj, getframe(gca)); 
    end
    
    drawnow;
end

if (makemovie)
    close(vidObj); 
end

figure(4);clf;hold on;
plot(errICP)
title('ICP Error 2-norm')
xlabel('Timestep')
ylabel('Error')