function plot_localization_state(figure_index, map, x, X, D)
    figure(figure_index);
    clf; 
    hold on;
    
    plot(map(:, 1), map(:, 2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(x(1, 1), x(2, 1), 'ro--')
    for dd = 1:D
        plot(X(1, dd), X(2, dd), 'b.')
    end
    
    axis equal
end
