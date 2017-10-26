function plot_errors(figure_index, T, x, mu_S, mu_Su, muP_S)
    figure(figure_index);
    clf; 
    hold on;
    
    % plot ekf
    e = sqrt((x(1, 2:end) - mu_S(1, 2:end)).^2+(x(3, 2:end)-mu_S(3, 2:end)).^2);
    plot(T(2:end), e, 'b', 'LineWidth', 1.5);
    
    % plot ukf
    eu = sqrt((x(1, 2:end) - mu_Su(1, 2:end)).^2 + (x(3, 2:end) - mu_Su(3, 2:end)).^2);
    plot(T(2:end), eu, 'g', 'LineWidth', 1.5);
    
    % plot pf
    ep = sqrt((x(1, 2:end) - muP_S(1, 2:end)).^2 +(x(3, 2:end) - muP_S(3, 2:end)).^2);
    plot(T(2:end), ep, 'm', 'LineWidth', 1.5);
    
    % plot details
    title('Position Estimation Errors for EKF, UKF and PF')
    xlabel('Time (s)');
    ylabel('X-Z Position Error (m)');
    legend('EKF', 'UKF', 'Particle');
end

