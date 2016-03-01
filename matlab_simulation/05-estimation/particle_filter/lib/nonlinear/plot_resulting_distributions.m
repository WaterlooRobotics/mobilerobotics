function plot_resulting_distributions(figure_index,  X, Y, yN, YN, pyN, yNG, pyNG, yLN, pyLN, yNU, pyNU, muN, muU, muLN)
    figure(figure_index); 
    clf; 
    hold on;
    
    plot(YN,pyN,'r', 'LineWidth', 1.5);
    plot(yNG,pyNG,'m--', 'LineWidth', 1.5);
    plot(yLN,pyLN,'g--','LineWidth', 1.5);
    plot(yNU,pyNU,'c--','LineWidth', 1.5);
    plot([muN muN],[0 max(pyNG)],'m--','LineWidth', 1.5)
    plot([muLN muLN], [0 max(pyLN)],'g--','LineWidth', 1.5)
    plot([muU muU], [0 max(pyNU)],'c--','LineWidth', 1.5)
    plot(X,0.001*ones(1,length(X)), 'cx', 'MarkerSize', 8, 'LineWidth', 2);
    plot(Y,0.001*ones(1,length(Y)), 'co', 'MarkerSize', 8, 'LineWidth', 2);
    
    title('Resulting Distribution')
    xlabel('y')
    ylabel('p(y)')
    legend('NL distribution', 'Best Gaussian Fit', 'EKF approx', 'UKF approx');
    axis(1.2*[2*min(yN) 2*max(yN) min(pyN) max(pyN)])
end

