% Laser scanner inverse measurement model plot
clc; clear;

M = 50;
N = 60;
m = 0.5*ones(M,N); %map

alpha = 1; % Distance about measurement to fill in
beta = 0.01; % Angle beyond which to exclude 

% Robot location
x = 25;
y = 10;
theta = pi/2;
rmax = 80; 

% Measurements
meas_phi = [-.4:0.01:.4]; % heading
meas_r = 40*ones(size(meas_phi)); % range

m = inversescanner(M,N,x,y,theta,meas_phi,meas_r,rmax,alpha,beta);

figure(2);clf;hold on;
image(100*(1-m));
plot(y,x,'kx','MarkerSize',8,'LineWidth',2)
% for i=1:length(meas_r)
%     plot( y+meas_r(i)*sin(meas_phi(i) + theta),x+meas_r(i)*cos(meas_phi(i)+ theta),'ko')
% end
colormap('gray')
axis equal