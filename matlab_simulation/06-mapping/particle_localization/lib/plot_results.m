function plot_results(figure_index, meas, map, t, x, X, y, mf, D)
    % plot setup
    figure(figure_index);
    clf; 
    hold on;
    
    % plot
    plot(map(:, 1), map(:,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(mf(1, t), mf(2, t),'mx', 'MarkerSize', 10, 'LineWidth', 2);
    plot(x(1, 1:t), x(2, 1:t), 'ro--');
    
    switch(meas)
    case 1  % 1 - range
        circle(1,x(1:2, t), y(1, t)); 
    case 2  % 2 - bearing
        plot([x(1, t) x(1, t) + 10 * cos(y(1, t) + x(3, t))], [x(2, t) x(2, t) + 10 * sin(y(1, t) + x(3, t))], 'c');
    case 3  % 3 - both
        plot([x(1, t) x(1, t) + y(1, t) * cos(y(2, t) + x(3, t))], [x(2, t) x(2, t) + y(1, t) * sin(y(2, t) + x(3, t))], 'c');
    end
    
    for dd = 1:D
     plot(X(1, dd), X(2, dd), 'b.')
    end
    
    % plot details
    axis equal
    axis([-5 7 -2 8]);
    title('Particle Filter Localization')
end

