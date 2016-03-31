function plot_figure(xd, T, index)
figure(index);
hold on;

% Select proper scale size to plot
switch index
    case 1
        scale = 0.05;
        title('Desired Trajectory: Spiral');
    case 2
        for t=1:5:length(T)
            drawcar(xd(1,t),xd(2,t),xd(3,t),.3,2);
        end
        scale = .3;
        title('Desired Trajectory: Nudges');
    case 3
        scale = .3;
        for t=1:5:length(T)
            drawcar(xd(1,t),xd(2,t),xd(3,t),.3,3);
        end
        title('Desired Trajectory: Swerves');
    case 4
        scale = .3;
        for t=1:5:length(T)
            drawcar(xd(1,t),xd(2,t),xd(3,t),.3,4);
        end
        title('Desired Trajectory: Corner');
    case 5
        scale = 0.05;
        title('Desired Trajectory: User defined');
end

plot(xd(1,:),xd(2,:));
% Plot a two-wheel robot along the trajectory
for t=1:5:length(T)
    drawcar(xd(1,t),xd(2,t),xd(3,t),scale,index);
end

axis equal;
end