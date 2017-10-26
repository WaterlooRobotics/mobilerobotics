function plot_figure(xd, T, index)
figure(index);
hold on;

% Select proper scale size to plot
switch index
    case 1
        scale = 0.05;
        title('Desired Trajectory: Spiral');
    case 2
        scale = .3;
        title('Desired Trajectory: Nudges');
    case 3
        scale = .3;
        title('Desired Trajectory: Swerves');
    case 4
        scale = .3;
        title('Desired Trajectory: Corner');
    case 5
        scale = .3;
        title('Desired Trajectory: A single lane change');
    case 6
        scale = .3;
        title('Desired Trajectory: Get around an obstacle');
        % Plot the obstacle
        ang=0:0.01:2*pi; 
        xp=1.5*cos(ang);
        yp=1.5*sin(ang);
        plot(7+xp,8.5+yp);
        hold on;
    case 7
        scale = 0.05;
        title('Desired Trajectory: User defined trajectory');
end

plot(xd(1,:),xd(2,:));
% Plot a two-wheel robot along the trajectory
for t=1:5:length(T)
    drawcar(xd(1,t),xd(2,t),xd(3,t),scale,index);
end

axis equal;
end
