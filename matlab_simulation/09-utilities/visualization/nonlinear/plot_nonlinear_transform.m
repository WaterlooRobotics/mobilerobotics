function plot_nonlinear_transform(figure_index, x, px, yN, pyN, YN, pyNG, yNG, muN)
    figure(figure_index);
    clf;
    
    subplot(2,2,4)
    plot(x, px, 'b');
    title('Original')
    xlabel('x')
    ylabel('p(x)')
    axis(1.2*[min(x) max(x) min(px) max(px)])
    
    subplot(2,2,2)
    plot(x, yN, 'g')
    title('Nonlinear Transformation y=tan^{-1}(x+1/2)')
    xlabel('x')
    ylabel('y')
    axis(1.2*[min(x) max(x) min(yN) max(yN)])

    subplot(2, 2, 1); 
    hold on;
    plot(pyN, YN, 'r');
    plot(pyNG, yNG, 'm--');
    plot([0 max(pyNG)], [muN muN], 'm--')
    axis(1.2*[min(pyN) max(pyN) min(yN) max(yN)])
    
    title('Resulting Distribution')
    xlabel('p(y)')
    ylabel('y')
end

