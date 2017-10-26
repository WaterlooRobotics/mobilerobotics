%% Plot function for FASTSLAM 
% INPUTS:
% -------
% None
% OUTPUTS:(in order of call)
% None
% --------
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Record movie if set so.
if(config.makemovie)
    vidObj = VideoWriter('fastslam.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 5;
    open(vidObj);
end
%% Color Map Intialization
cmap = colormap('jet');
cmap = cmap(1:3:end,:);
cn = length(cmap(:,1));
figure(1); hold on;

%% Define Axes
axis equal
axis([-10 10 -3 10])
title('FastSLAM with Range & Bearing Measurements');
%% Plot the Map
for j = 1:config.noFeatures
   LM=plot(map(1,j),map(2,j),'o','Color', cmap(mod(j,cn)+1,:), 'MarkerSize',10,'LineWidth',2);
end

%% Loop Through the DataSet and plot the results
for t=2:config.finalTime/config.timeInterval
    %Plot the Robot Pose
    L1=plot(pose(1,t),pose(2,t), 'ro--');
    %Plot the Robot True Pose
    L2=plot(truePose(1,t),truePose(2,t),'k^'); hold on
    % Draw the Heading
    L3=plot([pose(1,t) pose(1,t)+1*cos(pose(3,t))],...
        [pose(2,t) pose(2,t)+1*sin(pose(3,t))], 'r-');
    %Differentiate the currently Observed Features by drawing a line
    for j=1:length(measurements(t).indices)
       L4(j)=plot([pose(1,t) pose(1,t)+measurements(t).value(1,j)*cos(measurements(t).value(2,j)+pose(3,t))],...
            [pose(2,t) pose(2,t)+measurements(t).value(1,j)*sin(measurements(t).value(2,j)+pose(3,t))],...
            'Color', cmap(mod(measurements(t).indices(j),cn)+1,:) );
    end
    %Plot centroid of Particles
    L5=plot(centroid(1,t),centroid(2,t), 'g*');
    %Draw the Particles
    for d=1:config.totalParticles
        L6(d)=plot(particleSet(t).p(d).pose(1,1),particleSet(t).p(d).pose(2,1),'b.');
        for j = measurements(t).indices
           L7(d,j)= plot(particleSet(t).p(d).meanFeat(1,j),particleSet(t).p(d).meanFeat(2,j),'.','Color', cmap(mod(j,cn)+1,:));
        end
    end
    
    legend([L1,L2,L5],'Actual','No Noise','CentroidPose','Location','southoutside','Orientation','horizontal');
    if (config.makemovie) writeVideo(vidObj, getframe(gca)); end
    pause(0.001);
    %Deletes the old measurements and particles from the plot
    delete(L3)
    delete(L4)
    delete(L6)
    delete(L7) 
end
if (config.makemovie) close(vidObj); end

%Plot Errors
err_p=centroid-pose;
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
