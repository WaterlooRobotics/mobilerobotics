function[newPose,map,y,muParticleNew,covParticleNew,newParticleSet,meas_ind,newfeature,centroid_particles,w_new]...
        =fastSLAM2(oldPose,u,MotionNoise,MeasurementNoise,map,particleSet,muFeatOld,covFeatOld,w_new,rmax,thmax,timeInterval,totalParticles,newfeature)
    R=MeasurementNoise;Q=MotionNoise;noFeatures=size(map,2);covFeatPred=covFeatOld;muFeatPred=muFeatOld;
    %Move the robot forward
    newPose=motionUpdate_fs(oldPose,u,Q,timeInterval);
    %Look for features that are in FOV
    meas_ind=featureSearch_fs(noFeatures,map,newPose,rmax,thmax); 
    %Take Measurements
    y=[];
    for feature_index = meas_ind       
        y_new=measurementUpdate_fs(map(:,feature_index),newPose,R);
        y=[y,y_new];     
    end
 
    for particle=1:totalParticles
       %Move the Particles forward
       particleSetP(:,particle)=motionUpdate_fs(particleSet(:,particle),u,Q,timeInterval);
       clear yw hmuw Qw;
       %Run Individual EKFs for each particles to predict features
       for j=1:length(meas_ind)
           i = meas_ind(j);
           
%            if (newf):EKF else featupdate
            %Sample Proposal
           [mu_pose,cov_pose,H_pose,H_feat,pred_obs]=sample_proposal(...
               muFeatOld(:,i,particle),covFeatOld(:,:,i,particle),particleSetP(:,particle),y(:,j));

            [mu,cov,Hmu,Ht]=...
               ekf_fs(particleSetP(:,particle),y(:,j),muFeatOld(:,i,particle),...
               covFeatOld(:,:,i,particle),R,newfeature(i));
               muFeatPred(:,i,particle)=mu;covFeatPred(:,:,i,particle)=cov;
           %Accumulate the weights for the Particle Filter Update
           yw(2*(j-1)+1:2*j) = y(:,j);
           hmuw(2*(j-1)+1:2*j) = pred_obs;
%            Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) = 2*covFeatPred(:,:,i,particle);
       Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) =H_pose*Q*H_pose'+ H_feat*covFeatPred(:,:,i,particle)*H_feat'+R;
       end
       %Ensure that weights are not too low. 
       if (exist('Qw'))
            w_new(particle) = max(0.000001,mvnpdf(yw,hmuw,Qw));
       end
    end
        %Take note for all observed features.0:observed 1:New
        newfeature(meas_ind) = 0;
        %Run the RaoBlackwellization resampling step
        [muAllParticles,newParticleSet,muParticleNew,covParticleNew]=RB_fs(totalParticles,w_new,particleSetP,muFeatPred,covFeatPred);
        %Calculate the centroid of all the particles
        centroid_particles = muAllParticles;
        
%% Individual Function Definations Go Here.
   
    %% Motion Model
    function newPose=motionUpdate_fs(oldPose,controlInput,MotionNoise,dt)
    [QE, Qe] = eig(MotionNoise);n = length(MotionNoise(:,1)); 
     e = QE*sqrt(Qe)*randn(n,1);
    % Update robot state
        newPose = [oldPose(1,1)+controlInput(1,1)*cos(oldPose(3,1))*dt;% This means U is just a transla1ion and rotation
                  oldPose(2,1)+controlInput(1,1)*sin(oldPose(3,1))*dt;
                  oldPose(3,1)+controlInput(2,1)*dt] + e;
    end

    %% Measurement Model
    function obsFeat=measurementUpdate_fs(map,poseRobot,MeasNoise)
     [QiE, Qie] = eig(MeasNoise);
     m = length(MeasNoise(:,1)); % Number of measurements per feature 
     delta = QiE*sqrt(Qie)*randn(m,1);
     obsFeat=[sqrt((map(1,1)-poseRobot(1,1))^2 + (map(2,1)-poseRobot(2,1))^2);
                mod(atan2(map(2,1)-poseRobot(2,1),map(1,1)-poseRobot(1,1))-poseRobot(3,1)+pi,2*pi)-pi] + delta;

    end
    
    %% Feature Search
    function meas_ind=featureSearch_fs(M,map,xr,rmax,thmax)
        meas_ind = [];
        for k=1:M
                if(inview(map(:,k),xr,rmax,thmax))
                    meas_ind=[meas_ind,k];%concatenate the indices
                end
        end

            function yes = inview(f,x, rmax, thmax)
            % Checks for features currently in robot's view
            yes = 0;
            dx = f(1)-x(1);
            dy = f(2)-x(2);
            r = sqrt(dx^2+dy^2);
            th = mod(atan2(dy,dx)-x(3),2*pi);
            if (th > pi)
                th = th-2*pi;
            end
            if ((r<rmax) && (abs(th)<thmax))
                yes = 1;
            end
        end
    end
    
    %% Rao Blackwellization
    function [muAllParticles,newParticleSet,muParticleNew,covParticleNew]=RB_fs(totalParticles,w_new,particleSetP,muFeatPred,covFeatPred)
        W = cumsum(w_new);% Form the PDF and calculate the final cumulitive sum of all weights.
        % Resample and copy all data to new particle set
        for p=1:totalParticles
            seed = W(end)*rand(1);
            cur = find(W>seed,1);
            newParticleSet(:,p) = particleSetP(:,cur);
            muParticleNew(:,:,p) = muFeatPred(:,:,cur);
            covParticleNew(:,:,:,p) = covFeatPred(:,:,:,cur);
        end
        muAllParticles = mean(newParticleSet');
    end
    function [mu_pose,cov_pose,H_pose,H_feat,pred_obs]=sample_proposal(muFeatOld,covFeatOld,rob_pose,act_obs)
       dx = muFeatOld(1,1)-rob_pose(1,1);
       dy = muFeatOld(2,1)-rob_pose(2,1);
       rp = sqrt((dx)^2+(dy)^2);
       pred_obs=[rp; mod(atan2(dy,dx)-rob_pose(3,1)+pi,2*pi)-pi];
       H_feat = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];% Calculate Jacobian wrt features
       H_pose = [-dx/rp -dy/rp 0; -dy/rp^2 -dx/rp^2 -1];%Jacobian wrt Pose
       Q_mat=(R+H_feat*covFeatOld*H_feat');
       cov_pose=(H_pose'*((Q_mat)^-1)*H_pose+Q)^-1;
       mu_pose=cov_pose*H_pose'*(Q_mat^-1)*(act_obs-pred_obs)+rob_pose;
    end
    function [meanFeatP,covaFeatP]=addFeature(prior,obsFeat,H_feat,measUncertainty)
        meanFeatP(1,1) = prior(1,1)+obsFeat(1,1)*cos(obsFeat(2,1)+prior(3,1));
        meanFeatP(2,1) = prior(2,1)+obsFeat(1,1)*sin(obsFeat(2,1)+prior(3,1));
        covaFeatP= H_feat\measUncertainty*inv(H_feat)';%calculate cov
    end
    function [meanFeatP,covaFeatP]=ekf2(muOld,covOld,obsFeat,H_feat,pred_obs,R)
        I = obsFeat(:,1)-pred_obs;
        Q2 = Ht*covaFeat*H_feat' + measUncertainty;
        K = covaFeat*Ht'/Q2;
        meanFeatP = muOld + K*I;
        covaFeatP = (eye(2)-K*Ht)*covOld;
    end
end