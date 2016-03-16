function[newPose,map,y,muParticleNew,covParticleNew,newParticleSet,meas_ind,newfeature,centroid_particles,w_new]...
        =fastSLAM(oldPose,u,MotionNoise,MeasurementNoise,map,particleSet,muFeatOld,covFeatOld,w_new,rmax,thmax,timeInterval,totalParticles,newfeature)
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
           [mu,cov,Hmu]=...
               ekf_fs(particleSetP(:,particle),y(:,j),muFeatOld(:,i,particle),...
               covFeatOld(:,:,i,particle),R,newfeature(i));
               muFeatPred(:,i,particle)=mu;covFeatPred(:,:,i,particle)=cov;
           
           %Accumulate the weights for the Particle Filter Update
           yw(2*(j-1)+1:2*j) = y(:,j);
           hmuw(2*(j-1)+1:2*j) = Hmu;
%            Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) = 2*covFeatPred(:,:,i,particle);
       Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) = Hmu'*covFeatPred(:,:,i,particle)*Hmu+R;
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
        for i=1:M
                if(inview(map(:,i),xr,rmax,thmax))
                    meas_ind=[meas_ind,i];%concatenate the indices
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
        for particle=1:totalParticles
            seed = W(end)*rand(1);
            cur = find(W>seed,1);
            newParticleSet(:,particle) = particleSetP(:,cur);
            muParticleNew(:,:,particle) = muFeatPred(:,:,cur);
            covParticleNew(:,:,:,particle) = covFeatPred(:,:,:,cur);
        end
        muAllParticles = mean(newParticleSet');
    end
end