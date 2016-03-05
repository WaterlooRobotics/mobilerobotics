function plot_robot_path(x, t, fig)
figure(fig);
plot(x(2,1:t),x(1,1:t),'bx-')