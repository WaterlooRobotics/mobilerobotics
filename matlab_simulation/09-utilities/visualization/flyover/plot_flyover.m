function plot_flyover(figure_index, t, x, mu_S, mu_Su, muP_S, X)
    figure(figure_index);
    clf;
    hold on;
    
    % true state
    plot(x(1,2:t), x(3,2:t), 'ro--')
    
    % EKF
    plot(mu_S(1,2:t), mu_S(3,2:t), 'bx--')
    %plot_ellipse(mu, S);
    
    % UKF
    plot(mu_Su(1,2:t), mu_Su(3,2:t), 'gx--')
    %plot_ellipse(mu_Su, S_u);

    % PF
    plot(muP_S(1,2:t), muP_S(3,2:t), 'mx--')
    plot(X(1,1:10:end), X(3,1:10:end), 'm.')

    % plot ground
    plot(0, 0, 'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot([20 -1], [0 0],'b--')
    
    % plot details
    title('Flyover Example')
    axis equal
    axis([-5 20 -1 10])
    legend('True state', 'EKF', 'UKF', 'PF')
end

