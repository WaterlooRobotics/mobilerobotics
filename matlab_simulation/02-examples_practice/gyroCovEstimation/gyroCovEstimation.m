%% A Matlab program to calculate the convariances from a Gyroscope Data.
% It demonstrates the following aspects:
% ----Contributions in Ver.2.0
% -Vectorized the Gyro Data.
% -More Compact Code.
% -Removed repeatations
% -Completed the Error Distribution Comparison at the bottom.
% -Commented wherever possible
% 
% Author: Bismaya Sahoo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initializations
clear all; close all; clc;
RAD2DEG=180/pi;

tmin = 10; % Put Start_time here in seconds
tmax = 20; % Put Stop_time here in seconds

%% Load Data and Initial Calculations
load('gyroData.mat'); % Load the Gyro Dataset.
G_data=[gx gy gz]; % Vectorize the Dataset. G(:,1)=gx,G(:,2)=gy;G(:,3)=gz;
G_data=G_data*RAD2DEG; % Convert the Dataset to degrees.
dataSet_length=length(G_data(:,1));
time_Stamp=tg;

% Calculate Plotting interval
t_start= find(time_Stamp>tmin,1);% Find the start location in the dataset
t_stop= find(time_Stamp>tmax,1);% Find the stop location in the dataset

%% Calculate Covariance and A noisy simulation
% Assume underlying motion is negligible, and measurements are independent
S= cov(G_data);
[S_vect,S_val] = eig(S); % S_vect: eigVectors, S_val: eigValues

% Create a reproduction of gyro data
G_sim = S_vect*sqrt(S_val)*randn(3,dataSet_length);

% Plot the data
figure(1);clf;
subplot(2,1,1); hold on;grid on;
plot(time_Stamp(t_start:t_stop), G_data(t_start:t_stop,:));
axis([tmin tmax -60 60]);title('Original Data');xlabel('Time(s)');ylabel('Gyro Rate (deg/s)');legend('Gx','Gy','Gz')
subplot(2,1,2); hold on;grid on;
plot(tg(t_start:t_stop), G_sim(:,(t_start:t_stop)));
axis([tmin tmax -60 60]);xlabel('Time(s)');ylabel('Rate (deg/s)');title('Unfiltered simulation');legend('Gx_{Sim}','Gy_{Sim}','Gz_{Sim}');

%% Filter the Dataset 
filter_length = 3; % size of the moving avg filter.
G_filtered=filter(ones(1,filter_length)/filter_length,1,G_data); %Filter the Data.
G_err = G_data-G_filtered; %Calculate the error

Sf_err = cov(G_err);
[SErr_Vec, SErr_val]=eig(Sf_err);
% Create a reproduction of gyro data
G_simf = SErr_Vec*sqrt(SErr_val)*randn(3,dataSet_length);
% Plot the Filtered Data
figure(2);clf;
subplot(2,1,1); hold on;
plot(time_Stamp(t_start:t_stop), G_data(t_start:t_stop,:));
axis([tmin tmax -60 60]);title('Original Data');xlabel('Time(s)');ylabel('Gyro Rate (deg/s)');legend('Gx','Gy','Gz')
subplot(2,1,2); hold on;
plot(time_Stamp(t_start:t_stop), G_filtered(t_start:t_stop,:)'+G_simf(:,t_start:t_stop));
axis([tmin tmax -60 60]);xlabel('Time(s)');ylabel('Rate (deg/s)');title('Filtered simulation');legend('Gx_{Sim}','Gy_{Sim}','Gz_{Sim}');

%% Error Distribution Comparison
figure(3); clf; hold on
bins = 100;
xvec = -50:.1:50;
[Nex, Xex] = hist(G_data(:,1),bins);
[Ney, Xey] = hist(G_data(:,2),bins);
[Nez, Xez] = hist(G_data(:,3),bins);
binsizeX = Xex(2)-Xex(1);
binsizeY = Xey(2)-Xey(1);
binsizeZ = Xez(2)-Xez(1);

% Compare to pdf, by multiplying by number of samples and by bin size
% Num samples to convert to frequency distribution and bin size as smaller
% bins will have fewer samples.

Gex = normpdf(xvec,mean(G_data(:,1)),sqrt(Sf_err(1,1)))*length(G_err(:,1))*binsizeX;
Gey = normpdf(xvec,mean(G_data(:,2)),sqrt(Sf_err(2,2)))*length(G_err(:,2))*binsizeY;
Gez = normpdf(xvec,mean(G_data(:,3)),sqrt(Sf_err(3,3)))*length(G_err(:,3))*binsizeZ;

subplot(3,1,1);bar(Xex,Nex);hold on;plot(xvec,Gex,'r','LineWidth',2);
title('Comparison of Error Data and Gaussian Approximation');xlabel('Error Value');ylabel('Histogram Count');
subplot(3,1,2);bar(Xey,Ney);hold on;plot(xvec,Gey,'r', 'LineWidth',2);
title('Comparison of Error Data and Gaussian Approximation');xlabel('Error Value');ylabel('Histogram Count');
subplot(3,1,3);bar(Xez,Nez);hold on;plot(xvec,Gez,'r', 'LineWidth',2);
title('Comparison of Error Data and Gaussian Approximation');xlabel('Error Value');ylabel('Histogram Count');
 
