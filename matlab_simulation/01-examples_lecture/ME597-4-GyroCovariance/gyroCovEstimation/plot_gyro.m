%% Plot fucntion for Gyrro Data Estimation.
% AUTHOR: Bismaya Sahoo EMAIL:bsahoo@uwaterloo.ca
% COMMENTS: Non parametric function. No arguments. No returns.
% Didn't make sense to functionalize the whole thing because the fucntion
% would essentially result in as much clutter and wouldnt help in a cleaner
% code. Moving everything related to plot to a new file made more sense.
%----------------------------------------------------------------------
%% Plot Zone
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
