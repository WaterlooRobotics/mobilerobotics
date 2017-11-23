function plot_linear_transform(figure_index, x, px, yL, pyL)
    % Linear plot
    figure(figure_index);
    clf;

    % Original distribution
    subplot(2,2,4)
    plot(x, px, 'b');
    title('Original')
    xlabel('x')
    ylabel('p(x)')
    axis(1.2*[min(x) max(x) min(px) max(px)])

    % Transformation function
    subplot(2,2,2)
    plot(x, yL, 'g')
    title('Linear Transformation y=Ax+b')
    xlabel('x')
    ylabel('y')
    axis(1.2*[min(x) max(x) min(yL) max(yL)])
    axis equal

    % Resulting distribution
    subplot(2,2,1)
    plot(pyL,yL,'r');
    title('Final')
    xlabel('p(y)')
    ylabel('y')
    axis(1.2*[min(pyL) max(pyL) min(yL) max(yL)])
end

