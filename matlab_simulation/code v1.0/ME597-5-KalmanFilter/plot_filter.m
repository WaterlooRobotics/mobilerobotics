function plot_filter(x,y,t,mu_S,mu,S,example)

% This function plots each time step

if example == 2
    % Plot results
    figure(1);clf; hold on;
    plot(x(3,2:t),x(1,2:t), 'ro--')
    plot(y(2,2:t),y(1,2:t), 'gx')
    %plot(mup_S(3,1:t),mup_S(1,1:t), 'mx--')
    plot(mu_S(3,2:t),mu_S(1,2:t), 'bx--')
    mu_pos = [mu(3) mu(1)];
    S_pos = [S(3,3) S(3,1); S(1,3) S(1,1)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and beliefs')
    legend('State', 'Measurement','Estimate')
    axis equal
    axis([-10 10 -5 15])
end