clc; clear;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('Laser scanning.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 24;
    open(vidObj);
end

%% Depiction of map
% Assume there is a room with four obstacles
M = 50;
N = 60;
m = 0.5*ones(M,N); % map

% Obstacles
m(13:17,22:26)=0.6;  
m(1:8,55:60)=0.6;
m(25:30,40:43)=0.6;
m(30:34,5:7)=0.6;

%% Robot Motion
load('Robot Motion.mat'); % Pre-computed robot motion X = [x,y,theta]

%% Laser Parameters
rmax = 20; % Maximum range of laser scan
alpha = 1; % Distance about measurement to fill in
beta = 0.01; % Angle beyond which to exclude 

%% Measurements & Laser scanning
for t=1:139   % time of laser scanning 
% measurements
  meas_phi = [-.4:0.01:.4]; % heading
  meas_r = getranges(m,X(:,t),meas_phi,rmax); % range

% Laser scanning
  m2 = inversescanner(M,N,X(1,t),X(2,t),X(3,t),meas_phi,meas_r,rmax,alpha,beta);
  
  figure(1);clf;hold on;
  image(100*(1-m2));
  plot(X(2,t),X(1,t),'kx','MarkerSize',8,'LineWidth',2); % plotting robot position
  
 % plotting areas of have high possibility of obstacles (end points of laser beam)
    for i=1:length(meas_r) 
      plot( X(2,t)+meas_r(i)*sin(meas_phi(i) + X(3,t)),X(1,t)+meas_r(i)*cos(meas_phi(i)+ X(3,t)),'ko','MarkerSize',5);
    end
 
colormap('gray')
axis equal

    if (makemovie) writeVideo(vidObj, getframe(gca)); end 
end

if (makemovie) close(vidObj); end
