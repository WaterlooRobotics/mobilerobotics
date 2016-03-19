%% Fast SLAM2.0 function
% INPUTS:(in order of call)
% -------
% Robot's Current Pose
% Control Input
% Motion Noise
% Measurement Noise
% Old particle Set
% Mean of all observed Features
% covariance of all Observed Features
% Maximum Range
% Maximum Field of View
% time Interval
% total No of Particles 
% A vector with flags for observed and new features
% Set of weights or association index
% OUTPUTS:(in order of call)
% --------
% new Pose of the Robot
% Measurement Matrix
% New mean of The particle set
% New covariance of particles 
% new ParticleSet
% A vector containing all the measured indexs
% Updated vector with flags for newly observed features
% New Set of Weights or association Index
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%NOTE: The difference with FASTSLAM 1.0 is the sample_distribution before
%the observations and the way the weights are update. 

function[newPose,y,muParticleNew,covParticleNew,newParticleSet,meas_ind,newfeature,w_new]...
        =fastSLAM2(oldPose,u,MotionNoise,MeasurementNoise,map,particleSet,muFeatOld,covFeatOld,rmax,thmax,timeInterval,totalParticles,newfeature,w)
    R=MeasurementNoise;Q=MotionNoise;noFeatures=size(map,2);covFeatPred=covFeatOld;muFeatPred=muFeatOld;
    %Move the robot forward.This Motion has noise Q
    newPose=motionUpdate_fs(oldPose,u,Q,timeInterval);
    %Look for features that are in FOV
    meas_ind=featureSearch_fs(noFeatures,map,newPose,rmax,thmax); 
    %Take Measurements and accumulate them in a observation matrix
    y=[];
    for feature_index = meas_ind       
        y_new=measurementUpdate_fs(map(:,feature_index),newPose,R);
        y=[y,y_new];     
    end
    %Loop through all particles
    for particle=1:totalParticles
       %Move the Particles forward
       particleSetP(:,particle)=motionUpdate_fs(particleSet(:,particle),u,Q,timeInterval);
       clear yw hmuw Qw;
       %Run Individual EKFs for each particles to predict and correct features
       for j=1:length(meas_ind)
           i = meas_ind(j);
           
%          %Sample Proposal (new in 2.0)
           [mu_pose,cov_pose,H_pose,H_feat,pred_obs]=sample_proposal(...
               muFeatOld(:,i,particle),covFeatOld(:,:,i,particle),particleSetP(:,particle),y(:,j),R,Q);
           %CORRECTION OF FEATURES OR Addition of new features
           if(newfeature(i)==1)
                [mu,cov]=addFeature(particleSetP(:,particle),y(:,j),H_feat,R);
           else
                [mu,cov]=ekf_correct(muFeatOld(:,i,particle),covFeatOld(:,:,i,particle),y(:,j),H_feat,pred_obs,R);
           end
           muFeatPred(:,i,particle)=mu;covFeatPred(:,:,i,particle)=cov;
           %Accumulate the weights for the Particle Filter Update
           yw(2*(j-1)+1:2*j) = y(:,j);% target distribution
           hmuw(2*(j-1)+1:2*j) = pred_obs;% proposal distribution Mean
           %proposal distribution Covariance (new in 2.0)
           Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) =H_pose*Q*H_pose'+ H_feat*covFeatPred(:,:,i,particle)*H_feat'+R;
       end
       %Calculate the Weights and Ensure that they are not too low.
       if (exist('Qw'))
            w(particle) = max(0.000001,mvnpdf(yw,hmuw,Qw));
       end
    end
        %Take note for all observed features.0:observed 1:New
        newfeature(meas_ind) = 0;
        %Run the RaoBlackwellization resampling step
        [newParticleSet,muParticleNew,covParticleNew]=raoBlackwell_fs(totalParticles,w,particleSetP,muFeatPred,covFeatPred);
        w_new=w;
end