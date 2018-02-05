%% A Matlab program to calculate the convariances from a Gyroscope Data.
% It demonstrates the following aspects:
% -0. Load gyroDataset from the 11-datasets directory
% -1. Simulate a Gyro Data corrupted due to noise due to vehicle motion 
% -2. Filter the Simulated gyro data
% -3. Estimate the noise in the filtered data by comparing with original.
% -4. Add this noise to filtered data to verify it matches the original
% -5. Approximates the noise distribution 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initializations
clear all; close all; clc;
RAD2DEG = 180/pi;
tmin = 10; % Put Start_time here in seconds
tmax = 20; % Put Stop_time here in seconds

%% Load Data and Initial Calculations
load('gyroData.mat'); % Load the Gyro Dataset.
G_data = [gx gy gz]; % Vectorize the Dataset. G(:,1)=gx,G(:,2)=gy;G(:,3)=gz;
G_data = G_data*RAD2DEG; % Convert the Dataset to degrees.
% Assume underlying motion is negligible, and measurements are independent
% This is our ground truth.  Assume the original GyroData is accurate.

S= cov(G_data);%Calculate the Covariance for reproduction of simulated data
fprintf('The Covariance of the Original Dataset is:\n');disp(S);
[S_vect,S_val] = eig(S); % S_vect: eigVectors, S_val: eigValues

dataSet_length = length(G_data(:,1));
time_Stamp = tg;%tg is the variable name for timeStamp in Data File.
% Calculate Plotting interval
t_start = find(time_Stamp>tmin,1);% Find the start location in the dataset
t_stop = find(time_Stamp>tmax,1);% Find the stop location in the dataset

%% Run the process/simulation
% STEP1: Simulate Gyro Data that includes vehicle motions
% The reproduction would have the same cov but corrupted by noise.
G_sim = S_vect*sqrt(S_val)*randn(3,dataSet_length);

% STEP2: Filter the Gyro Sensor on our robot with a moving avg filter.
filter_length = 3; % size of the moving avg filter.
G_filtered=filter(ones(1,filter_length)/filter_length,1,G_data); %Filter the Data.

% STEP3: Calculate the error of our filtered estimate from the original data.
% This error when added to our estimate should give estimated gyro data.
G_err = G_data-G_filtered; %Calculate the error
Sf_err = cov(G_err); % Calculate the error covariance
fprintf('The Covariance of the Noise added to the filtered dataset is:\n');disp(Sf_err);
[SErr_Vec, SErr_val]=eig(Sf_err);

% STEP 4: Create a reproduction of the error
G_sim_err = SErr_Vec*sqrt(SErr_val)*randn(3,dataSet_length);

% STEP 5: Calculate our final Gyro Estimate.
G_out=G_filtered'+G_sim_err;

% STEP 6: Estimate the Error Distribution
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

%% Plot everything.

% Plot the Original data
figure(1);clf;
subplot(2,2,1); hold on;grid on;
plot(time_Stamp(t_start:t_stop), G_data(t_start:t_stop,:));
axis([tmin tmax -60 60]);title('1.Original Data');xlabel('Time(s)');ylabel('Gyro Rate (deg/s)');legend('Gx','Gy','Gz')
%Plot the Simulated Data with vehicle motions
subplot(2,2,2); hold on;grid on;
plot(tg(t_start:t_stop), G_sim(:,(t_start:t_stop)));
axis([tmin tmax -60 60]);title('2.Simulated Data');xlabel('Time(s)');ylabel('Gyro Rate (deg/s)');legend('Gx_{Sim}','Gy_{Sim}','Gz_{Sim}');
% Plot the Filtered Data
subplot(2,2,3); hold on;grid on;
plot(time_Stamp(t_start:t_stop), G_filtered(t_start:t_stop,:));
axis([tmin tmax -60 60]);title('3.Filtered Data');xlabel('Time(s)');ylabel('Gyro Rate (deg/s)');legend('Gx_f','Gy_f','Gz_f')
% Plot the Predicted Gyro Data
subplot(2,2,4); hold on;grid on;
plot(time_Stamp(t_start:t_stop), G_out(:,t_start:t_stop));
axis([tmin tmax -60 60]);title('4.Estimated Data');xlabel('Time(s)');ylabel('Gyro Rate (deg/s)');legend('Gx_{pred}','Gy_{pred}','Gz_{pred}');

% Plot the Error Distribution
figure(2); clf; hold on
subplot(3,1,1);bar(Xex,Nex);hold on;plot(xvec,Gex,'r','LineWidth',2);
title('Error Distribution in X');xlabel('Error Value');ylabel('Histogram Count');
subplot(3,1,2);bar(Xey,Ney);hold on;plot(xvec,Gey,'r', 'LineWidth',2);
title('Error Distribution in Y');xlabel('Error Value');ylabel('Histogram Count');
subplot(3,1,3);bar(Xez,Nez);hold on;plot(xvec,Gez,'r', 'LineWidth',2);
title('Error Distribution in Z');xlabel('Error Value');ylabel('Histogram Count');

