%% A Matlab program to calculate the convariances from a Gyroscope Data.
% It demonstrates the following aspects:
% -1. Load gyroDataset from the 11-datasets directory
% -2. Adds Gaussian Noise to the dataset
% -3. Filters the Dataset using a moving average filter
% -4. Displays both the noisy and the filtered datasets for comparision
% -5. Approximates the noise distribution that was added to the dataset
% ---------------------------------------------------------------------
% ----Contributions in Ver.2.0
% -1. Vectorized the Gyro Data.
% -2. More Compact Code.
% -3. Removed repeatations
% -4. Completed the Error Distribution Comparison at the bottom.
% -5. Commented wherever possible
% ----Contributions in Ver.3.0 after review
% -1. Updated the MobileRoboticsSetup.m to add the 11-datasets folder
% -2. Removed a duplicate copy of this same file
% -3. Added the Info about this file in the top comments
% -4. Changed the titles of the error plots which were earlier the same.
% -5.  
% Author: Bismaya Sahoo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initializations
clear all; close all; clc;
RAD2DEG = 180/pi;

tmin = 10; % Put Start_time here in seconds
tmax = 20; % Put Stop_time here in seconds

%% Load Data and Initial Calculations
load('gyroData.mat'); % Load the Gyro Dataset.
G_data = [gx gy gz]; % Vectorize the Dataset. G(:,1)=gx,G(:,2)=gy;G(:,3)=gz;
G_data = G_data*RAD2DEG; % Convert the Dataset to degrees.
dataSet_length = length(G_data(:,1));
time_Stamp = tg;

% Calculate Plotting interval
t_start = find(time_Stamp>tmin,1);% Find the start location in the dataset
t_stop = find(time_Stamp>tmax,1);% Find the stop location in the dataset

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
[Nex, Xex] = hist(G_err(:,1),bins);
[Ney, Xey] = hist(G_err(:,2),bins);
[Nez, Xez] = hist(G_err(:,3),bins);
binsizeX = Xex(2)-Xex(1);
binsizeY = Xey(2)-Xey(1);
binsizeZ = Xez(2)-Xez(1);

% Compare to pdf, by multiplying by number of samples and by bin size
% Num samples to convert to frequency distribution and bin size as smaller
% bins will have fewer samples.

Gex = normpdf(xvec,mean(G_err(:,1)),sqrt(Sf_err(1,1)))*length(G_err(:,1))*binsizeX;
Gey = normpdf(xvec,mean(G_err(:,2)),sqrt(Sf_err(2,2)))*length(G_err(:,2))*binsizeY;
Gez = normpdf(xvec,mean(G_err(:,3)),sqrt(Sf_err(3,3)))*length(G_err(:,3))*binsizeZ;

subplot(3,1,1);bar(Xex,Nex);hold on;plot(xvec,Gex,'r','LineWidth',2);
title('Error Distribution in X');xlabel('Error Value');ylabel('Histogram Count');
subplot(3,1,2);bar(Xey,Ney);hold on;plot(xvec,Gey,'r', 'LineWidth',2);
title('Error Distribution in Y');xlabel('Error Value');ylabel('Histogram Count');
subplot(3,1,3);bar(Xez,Nez);hold on;plot(xvec,Gez,'r', 'LineWidth',2);
title('Error Distribution in Z');xlabel('Error Value');ylabel('Histogram Count');
 
