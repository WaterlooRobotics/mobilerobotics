%% RUN THE FAST SLAM FUNCTION
% INPUTS:(in order of call)
% -------
% None
% OUTPUTS:(in order of call)
% --------
% None
% INSTRUCTIONS:
%-------------
% 1.Changes can be made in the initialization section.(Straightforward)
% 2.UNCOMMENT specified sections to run FAST SLAM1.0 or FAST SLAM2.0 and
% COMMENT the other.
% 3.Set randMapFlag to 0 to load the static map.(staticMap.m)must
% exist.This is useful for comparision of fastSLAM1.0 and 2.0. Setting this
% value to 1 randomizes the map in each iteration.
% 
% NOTE: Advantages of the fastSLAM2.0 can be observed by reducing the no of
% features, increassing the motion noise, increasing total features
% COMMENTS:
% 1.TruePose(BLACK): here refers to the pose,we want the robot to undertake.But it
% reality it is corrupted by motion noise(RED).This is just for reference.
% The robot in reality executes the red trajectory which we have to track
% using fastSLAM.
% 2. Plotting takes most of the time.
% 3. The movie making is currently disabled.But can be added.
% 4. RMS error displayed at the end in prompt.
% 5. Q=Motion Noise and R=Measurement Noise, unlike the slides(flipped).
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initializations
clc; clear all; close all; 
% Primary Initializations           
poseInitial=[0,0,0]';%initial Pose
dt=0.1; timeInterval=dt;%time interval
finalTime=20;%final Time in sec
MotionNoise=[0.001 0 0;
             0 0.001 0;
             0 0 0.0001];Q=MotionNoise;
MeasurementNoise=[0.02 0;
                 0 0.02];R=MeasurementNoise;
noFeatures=20;% No of features in Map
totalParticles=100;%No of particles
rmax = 10;
thmax = pi/4;
u=[1;0.3];

%Fast Slam Specific Initializations
pose(:,1)=poseInitial;
particleSet=zeros(size(poseInitial,1),totalParticles);
muFeat=zeros(size(R,1),noFeatures,totalParticles);
muFeatPred=muFeat;
covFeat=zeros(size(R,1),size(R,1),noFeatures,totalParticles);
covFeatPred=covFeat;
newfeature = ones(noFeatures,1);
w_initial=1/totalParticles;
w=w_initial*ones(1,totalParticles);
timeStamp(1)=timeInterval; 
truePose(:,1)=poseInitial;
%Map Initializations
randMapFlag=0;% Set=1:to randomize Map. 0:to load static map.(staticMap.m must exist)
if(randMapFlag==1)
map = 10*rand(2,noFeatures);
map(1,:) = map(1,:)-5; 
map(2,:) = map(2,:)-2; 
else
    load('staticMap.mat');
end
%% Fast SLAM LOOP
for t=2:finalTime/timeInterval
    oldPose=pose(:,t-1);
    if(t==2)
        oldParticleSet=particleSet;
        w_old=w;
        muFeatOld=muFeat;
        covFeatOld=covFeat;
    end
   %% % %     UNCOMMENT to call the fast slam1.0 function
%     [newPose,y,muFeatNew,covFeatNew,newParticleSet,meas_ind,newfeature,w_new]...
%         =func_fastSLAM(oldPose,u,MotionNoise,MeasurementNoise,map,oldParticleSet,muFeatOld,covFeatOld,rmax,thmax,timeInterval,totalParticles,newfeature,w_old);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     

%% % %     UNCOMMENT to call the fast2.0 slam function
    [newPose,y,muFeatNew,covFeatNew,newParticleSet,meas_ind,newfeature,w_new]...
        =func_fastSLAM2(oldPose,u,MotionNoise,MeasurementNoise,map,oldParticleSet,muFeatOld,covFeatOld,rmax,thmax,timeInterval,totalParticles,newfeature,w_old);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

    %STORE the Pose,Centroid and timestamp
    timeStamp(t)=timeStamp(t-1)+timeInterval;
    pose(:,t)=newPose;
    centroid(:,t)=mean(newParticleSet');
    
    %Updates the true pose of the robot
    truePose(:,t)=motionUpdate_fs(truePose(:,t-1),u,zeros(3),timeInterval);
    %Plot the Results
    plot_fs(pose,map,y,muFeatNew,newParticleSet,t,meas_ind,totalParticles,noFeatures,centroid,truePose,finalTime/timeInterval);
    
    % Swap for the next iteration        
    oldParticleSet=newParticleSet;
    muFeatOld=muFeatNew;
    covFeatOld=covFeatNew;
    w_old=w_new;
    
end
