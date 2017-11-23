function plot_sample_weights(figure_index, M, x, fx, gx, xP, wx)
    figure(figure_index);
    clf;
    subplot(2,1,1); 
    hold on;

    plot(x, fx / 0.001, 'b');
    plot(x, gx / 0.001, 'r');
    plot(x, fx ./ gx, 'g');

    title('Weights for each sample of g(x)')
    legend('f(x)', 'g(x)', 'f(x)/g(x)')
    axis([-3 2 0 7])
    subplot(2,1,2); 
    hold on;

    for m = 1:25:M
        plot([xP(m); xP(m)], [0 wx(m)], 'g')
    end
    xlabel('x')
    ylabel('Weights f(x)/g(x)')
    axis([-3 2 0 max(wx)])
end