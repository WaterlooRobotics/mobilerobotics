function plot_prediction(figure_index, L, mu, mup, S, Sp, Xp)
    figure(figure_index);
    clf;
    subplot(2,1,1);
    hold on;
    
    z = [(mu - L * sqrt(S)):(0.01):(mu + L * sqrt(S))];
    plot(z, normpdf(z, mu, S), 'b');
    
    z = [(mup - L * sqrt(Sp)):(0.01):(mup + L * sqrt(Sp))];
    plot(z, normpdf(z, mup, Sp),'r');
    
    title('Prior & Prediction')
    legend('Prior', 'Prediction')
    axis([0 15 0 0.4])
    subplot(2, 1, 2);
    hold on;
    for m = 1:length(Xp)
        plot([Xp(m); Xp(m)], [0 1], 'r')
    end
    axis([0 15 0 1])
end