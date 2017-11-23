function plot_prior_belief(figure_index, L, mu, S, X0)
    figure(figure_index);
    clf;
    subplot(2,1,1); 
    hold on;
    
    z = [(mu - L * sqrt(S)):(0.01):(mu + L * sqrt(S))];
    plot(z, normpdf(z, mu, S), 'b');
    plot([0 15], [1/15 1/15])
    
    title('Priors')
    axis([5 15 0 0.4])
    subplot(2, 1, 2);
    hold on;
    for m = 1:length(X0)
        plot([X0(m); X0(m)], [0 1], 'b')
    end
    axis([5 15 0 1])
end
