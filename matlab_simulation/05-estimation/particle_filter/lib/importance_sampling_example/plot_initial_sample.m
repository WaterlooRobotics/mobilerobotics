function plot_initial_sample(figure_index, M, x, fx, gx, xP)
    figure(figure_index);
    clf;
    
    % plot of both distributions, and initial sample of g(x)
    % f(x) and g(x)
    subplot(3,1,1); 
    hold on;
    plot(x, fx / 0.01, 'b');
    plot(x, gx / 0.01, 'r');
    title('Distributions, and Samples from g(x)')
    legend('f(x)', 'g(x)')
    axis([-3 2 0 0.7])

    % actual samples plotted as bars, solidity represents density
    subplot(3, 1, 2); 
    hold on;
    for m=1:25:M
        plot([xP(m);xP(m)],[0 1],'r')
    end
    ylabel('Samples');
    axis([-3 2 0 1])

    % histogram of samples into 200 bins
    subplot(3,1,3); 
    hold on;
    [xPH, xH] = hist(xP, 200);
    plot(xH, xPH ./ M, 'r');
    xlabel('x');
    ylabel('# samples');
    axis([-3, 2, 0, 1.1 * max(xPH ./ M)])
end

