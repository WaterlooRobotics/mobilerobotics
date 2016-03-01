function plot_measurement(figure_index, t, L, mu, mup, S, Sp, Q, X, y)
    figure(figure_index);
    clf;
    subplot(2,1,1);
    hold on;
    
    z = [(mup - L * sqrt(Sp)):(0.01):(mup + L * sqrt(Sp))];
    plot(z, normpdf(z, mup, Sp), 'r');
    
    z = [(y(t) - L * sqrt(Q)):(0.01):(y(t) + L * sqrt(Q))];
    plot(z, normpdf(z, y(t), Q), 'g');
    
    z = [(mu - L * sqrt(S)):(0.01):(mu + L * sqrt(S))];    
    plot(z,normpdf(z, mu, S), 'm');
    
    axis([0 15 0 0.6]);
    title('Prediction, Measurement & Belief')
    legend('Prediction','Measurement', 'Belief')
    subplot(2,1,2); hold on;
    for m = 1:length(X)
        plot([X(m); X(m)], [0 1], 'm')
    end
    axis([0 15 0 1]);
end