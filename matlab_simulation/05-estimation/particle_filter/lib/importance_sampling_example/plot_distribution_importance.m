function plot_distribution_importance(figure_index, M, x, fx, gx, xPnew)
    figure(figure_index);
    clf;
    
    subplot(2,1,1); 
    hold on;

    plot(x, fx/0.01,'b');
    plot(x, gx/0.01,'r');

    title('Distributions, and importance sampled f(x)')
    legend('f(x)', 'g(x)')
    axis([-3 2 0 0.7])

    subplot(2,1,2); 
    hold on;
    [xPnewH, xnewH] = hist(xPnew, 200);
    plot(xnewH, xPnewH ./ M, 'b');
    
    xlabel('x');
    ylabel('# samples');
    axis([-3, 2, 0, 1.1 * max(xPnewH ./ M)])
end

