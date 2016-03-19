%% PLOT FUNCTION FOR FAST SLAM AND FAST SLAM2.0
% INPUTS:(in order of call)
% -------
% Robot's Current Pose
% Environment MAP
% Observation Matix
% Mean of all observed Features
% Particle Set
% Current time index
% A vector with indices of observed features
% Total no of particles
% Total no of features
% centroid Loacations of the Particles
% True Pose of the robot
% OUTPUTS:(in order of call)
% --------
% None
% % AUTHOR: BISMAYA SAHOO, EMAIL:bsahoo@uwaterloo.ca
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function[]=plot_fs(pose,map,y,muFeatNew,particleSet,t,meas_ind,totalParticles,noFeatures,centroid_particles,truePose,tf)
        %Color Map Intialization
        cmap = colormap('jet');
        cmap = cmap(1:3:end,:);
        cn = length(cmap(:,1));
        figure(1);clf; hold on;
        %Plot the Robot Pose
        l(1)=plot(pose(1,1:t),pose(2,1:t), 'ro--');
        %Plot the Robot Pose
        l(2)=plot(truePose(1,1:t),truePose(2,1:t),'k','LineWidth',3);
        % Draw the Heading
        plot([pose(1,t) pose(1,t)+1*cos(pose(3,t))],[pose(2,t) pose(2,t)+1*sin(pose(3,t))], 'r-')
        %Differentiate the currently Observed Features by drawing a line
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
            for j = meas_ind
                plot(muFeatNew(1,j,d),muFeatNew(2,j,d),'.','Color', cmap(mod(j,cn)+1,:));
            end
        end
        %Plot Centroid of Particles
        l(3)=plot(centroid_particles(1,1:t),centroid_particles(2,1:t), 'g*');

        %Define Axes
        axis equal
        axis([-10 10 -3 10])
        title('FastSLAM with Range & Bearing Measurements');
        legend(l(1:3),'Actual','No Noise','CentroidPose','Location','southoutside','Orientation','horizontal');
        pause(0.001);%Pause to let MATLAB plot and display the results
        
        if(t==tf)
            err_p=centroid_particles-pose;
            %Error Between true pose and fast slam prediction
            figure(2);
            subplot(3,1,1);
            plot(err_p(1,:));hold on;title('Error Plots');xlabel('time');ylabel('X_{err}');
            subplot(3,1,2);
            plot(err_p(2,:));hold on;xlabel('time');ylabel('Y_{err}');
            subplot(3,1,3);
            plot(err_p(3,:));hold on;xlabel('time');ylabel('\theta_{err}');
            fprintf('RMS error in X_pos: %f \n',rms(err_p(1,:)));
            fprintf('RMS error in Y_pos: %f \n',rms(err_p(2,:)));
            fprintf('RMS error in Orientation: %f \n',rms(err_p(3,:)));
        end
end