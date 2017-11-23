%% Fast SLAM2.0 function
% INPUTS:(in order of call)
% -------
% Robot's Current Pose
% Control Input
% Configurations
% Map
% Old particle Set
% OUTPUTS:(in order of call)
% --------
% Predictd Pose of the Robot
% Measurements  
% new ParticleSet
% Updated vector with flags for newly observed features
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%NOTE: The difference with FASTSLAM 1.0 is the sample_distribution before
%the observations and the way the weights are update. 

function[predPose,measurements,particles_new,newfeature]...
        =func_fastSLAM2(oldPose,u,config,map,particles_old)
        
    R=config.MeasNoise;Q=config.MotNoise;noFeatures=size(map,2);
    rmax=config.maxRange;thmax=config.maxFOV;dt=config.timeInterval;
    totalParticles=config.totalParticles;newfeature=config.newFeat;
    
    %Move the robot forward.This Motion has noise Q
    predPose=motionUpdate_fs(oldPose,u,Q,dt);
    %Look for features that are in FOV
    meas_ind=featureSearch_fs(noFeatures,map,predPose,rmax,thmax); 
    %Take Measurements and accumulate them in a observation matrix
    y=[];
    for feature_index = meas_ind       
        y_new=measurementUpdate_fs(map(:,feature_index),predPose,R);
        y=[y,y_new];     
    end
    
    measurements.indices=meas_ind;measurements.value=y;
    %Loop through all particles
    for particle=1:totalParticles
       %Move the Particles forward
        particles_old(particle).pose=motionUpdate_fs(particles_old(particle).pose,u,Q,dt);
       clear yw hmuw Qw;
       %Run Individual EKFs for each particles to predict and correct features
       for j=1:length(meas_ind)
           i = meas_ind(j);
           
%          %Sample Proposal (new in 2.0)
           [mu_pose,cov_pose,H_pose,H_feat,pred_obs]=sample_proposal(...
               particles_old(particle).meanFeat(:,i),...
               particles_old(particle).covFeat(:,:,i),...
               particles_old(particle).pose,y(:,j),R,Q);
           %CORRECTION OF FEATURES OR Addition of new features
           if(newfeature(i)==1)
                [mu,cov]=addFeature(particles_old(particle).pose,y(:,j),H_feat,R);
           else
                [mu,cov]=ekf_correct(particles_old(particle).meanFeat(:,i),...
                    particles_old(particle).covFeat(:,:,i),...
                    y(:,j),H_feat,pred_obs,R);
           end
           particles_old(particle).meanFeat(:,i)=mu;
           particles_old(particle).covFeat(:,:,i)=cov;
           %Accumulate the weights for the Particle Filter Update
           yw(2*(j-1)+1:2*j) = y(:,j);% target distribution
           hmuw(2*(j-1)+1:2*j) = pred_obs;% proposal distribution Mean
           %proposal distribution Covariance (new in 2.0)
           Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) =H_pose*Q*H_pose'+...
               H_feat*particles_old(particle).covFeat(:,:,i)*H_feat'+R;
       end
       
       %Calculate the Weights and Ensure that they are not too low.
       if (exist('Qw1'))
            Qw1=round(Qw,6);% To avoid error in cholcov in mvnpdf.
            w(particle) = max(0.000001,mvnpdf(yw,hmuw,Qw1));
       end
    end
        %Take note for all observed features.0:observed 1:New
        newfeature(meas_ind) = 0;
        %Run the RaoBlackwellization resampling step
        [particles_new]=raoBlackwell_fs(totalParticles,particles_old);
end