function plot_trajectory(figure_index, T, x, y, mu_S, muP_S, M)
    figure(figure_index);
    clf;
    hold on;
    
    plot(T, x(2:end), 'b')
    %plot(T,y,'rx')
    %plot(T,u,'g');
    plot(T, mu_S, 'r--')
    plot(T, muP_S, 'co--')
    plot(T, 2 * ones(size(T)), 'm--');
    plot(T, 10 * ones(size(T)), 'm--');
    
    title(sprintf('State and estimates, M=%d', M));
    legend('State', 'Kalman Estimate', 'Particle Estimate')
end
