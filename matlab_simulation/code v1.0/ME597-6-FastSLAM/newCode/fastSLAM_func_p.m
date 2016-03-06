function []=fastSLAM_func(PoseInitial,MotionNoise,MeasurementNoise,MapSize,FinalTime,dt,ParticleSize)
totalFeatures=MapSize;R=MeasurementNoise;Q=MotionNoise;


[timeInterval,finalTime,noFeatures,map,pose,totalParticles,muFeat,muFeatPred,covFeat,covFeatPred,newfeature,w,u,timeStamp,rmax,thmax]...
            =initializeFastSLAM(PoseInitial,MotionNoise,MeasurementNoise,MapSize,FinalTime,dt,ParticleSize);
%% Fast SLAM 
figure(1); 
%Plot the Map

for t=2:finalTime/timeInterval
    timeStamp(t)=timeStamp(t-1)+timeInterval;
    pose(:,t)=motionUpdate_fs(pose(:,t-1),u,Q,timeInterval);
    
    meas_ind=featureSearch_fs(noFeatures,map,pose(:,t),rmax,thmax); 
    y=[];
    for feature_index = meas_ind       
        y_new=measurementUpdate_fs(map(:,feature_index),pose(:,t),R);
        y=[y,y_new];     
    end

    for particle=1:totalParticles
       particleSetP(:,particle)=motionUpdate_fs(particleSet(:,particle),u,Q,timeInterval);
       clear yw hmuw Qw;
       for j=1:length(meas_ind)
           i = meas_ind(j);
           [muFeatPred(:,i,particle),covFeatPred(:,:,i,particle),Hmu]=...
               ekf_FS1(particleSetP(:,particle),y(:,j),muFeat(:,i,particle),...
               covFeat(:,:,i,particle),R,newfeature(i));
           yw(2*(j-1)+1:2*j) = y(:,j);
           hmuw(2*(j-1)+1:2*j) = Hmu;
           Qw(2*(j-1)+1:2*j,2*(j-1)+1:2*j) = 2*covFeatPred(:,:,i,particle);
       end
       if (exist('Qw'))
            w(particle) = max(0.000001,mvnpdf(yw,hmuw,Qw));
       end
    end
        newfeature(meas_ind) = 0; 
        [muParticle,particleSet,muFeat,covFeat]=raoBlackwellFilter_fs(totalParticles...
            ,w,particleSetP,muFeatPred,covFeatPred);
        mu_S(:,t) = muParticle;
        
        
        
     plot_fs(pose,map,y,muFeat,particleSet,t,meas_ind,totalParticles,noFeatures,newfeature);
       pause(0.1);
end



    %% Initialization Function   
    function[timeInterval,finalTime,noFeatures,map,pose,totalParticles,muFeat,muFeatPred,covFeat,covFeatPred,newfeature,w,u,timeStamp,rmax,thmax]...
            =initializeFastSLAM(PoseInitial,MotionNoise,MeasurementNoise,MapSize,FinalTime,dt,ParticleSize)
        poseInitial=PoseInitial;
        timeInterval=dt;
        finalTime=FinalTime;%final Time in sec
        noFeatures=MapSize;
        map = 10*rand(2,noFeatures);
        map(1,:) = map(1,:)-5; 
        map(2,:) = map(2,:)-2; 
        Q=MeasurementNoise;R=MotionNoise;
%         Q=[0.001 0 0;0 0.001 0;0 0 0.0001];
%         R=[0.02 0;0 0.02];
        rmax = 10;
        thmax = pi/4;
        pose(:,1)=poseInitial;
        totalParticles=ParticleSize;
        particleSet=zeros(size(poseInitial,1),totalParticles);
        measStateSize=size(R,1);
        poseSize=size(Q,1);
        muFeat=zeros(measStateSize,noFeatures,totalParticles);
        muFeatPred=muFeat;
        covFeat=zeros(measStateSize,measStateSize,noFeatures,totalParticles);
        covFeatPred=covFeat;
        newfeature = ones(totalParticles,1);
        w_initial=1/totalParticles;
        w=w_initial*ones(1,totalParticles);
        u=[2;0.3];
        timeStamp(1)=timeInterval; 
    end

    %% Motion Model
    function xr=motionUpdate_fs(xr,u,Q,dt)
    [QE, Qe] = eig(Q);n = length(Q(:,1)); 
     e = QE*sqrt(Qe)*randn(n,1);
    % Upda1e robot state
        xr = [xr(1,1)+u(1,1)*cos(xr(3,1))*dt;% This means U is just a transla1ion and rotation
                  xr(2,1)+u(1,1)*sin(xr(3,1))*dt;
                  xr(3,1)+u(2,1)*dt] + e;
    end

    %% Measurement Model
    function y_meas=measurementUpdate_fs(map,xr,Qi);
     [QiE, Qie] = eig(Qi);
     m = length(Qi(:,1)); % Number of measurements per feature 
     delta = QiE*sqrt(Qie)*randn(m,1);
     y_meas=[sqrt((map(1,1)-xr(1,1))^2 + (map(2,1)-xr(2,1))^2);
                mod(atan2(map(2,1)-xr(2,1),map(1,1)-xr(1,1))-xr(3,1)+pi,2*pi)-pi] + delta;

    end

    %% Rao Blackwellization
    function [muParticle,X,mu,S]=raoBlackwellFilter_fs(D,w,Xp,mup,Sp)
     % Resampling
        W = cumsum(w);
        % Resample and copy all data to new particle set
        for d=1:D
            seed = W(end)*rand(1);
            cur = find(W>seed,1);
            X(:,d) = Xp(:,cur);
            mu(:,:,d) = mup(:,:,cur);
            S(:,:,:,d) = Sp(:,:,:,cur);
        end

        muParticle = mean(X');
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
            % Checks if a feature is in view
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

    %% Plot for FastSLAM
    function[]=plot_fs(pose,map,y,muFeat,particleSet,t,meas_ind,totalParticles,noFeatures,newfeature)
        %Color Map Intialization
        cmap = colormap('jet');
        cmap = cmap(1:3:end,:);
        cn = length(cmap(:,1));
        figure(1);clf; hold on;
%         %Plot the Robot Pose
         plot(pose(1,1:t),pose(2,1:t), 'ro--')
        % Draw the Laser Line
        plot([pose(1,t) pose(1,t)+1*cos(pose(3,t))],[pose(2,t) pose(2,t)+1*sin(pose(3,t))], 'r-')
        %Differentiate the Observed Features
        for j=1:length(meas_ind)
            plot([pose(1,t) pose(1,t)+y(1,j)*cos(y(2,j)+pose(3,t))], [pose(2,t) pose(2,t)+y(1,j)*sin(y(2,j)+pose(3,t))], 'Color', cmap(mod(meas_ind(j),cn)+1,:) );
        end
        % Plot the Map
        for j = 1:noFeatures
            plot(map(1,j),map(2,j),'o','Color', cmap(mod(j,cn)+1,:), 'MarkerSize',10,'LineWidth',2);
        end
        %Draw the Particles
        for d=1:totalParticles
            plot(particleSet(1,d),particleSet(2,d),'b.');
            for j = 1:noFeatures
                if (~newfeature(j))
                    plot(muFeat(1,j,d),muFeat(2,j,d),'.','Color', cmap(mod(j,cn)+1,:));
                end
            end
        end
        %Define Axes
        axis equal
        axis([-8 8 -2 10])
        title('FastSLAM with Range & Bearing Measurements')
    end

end