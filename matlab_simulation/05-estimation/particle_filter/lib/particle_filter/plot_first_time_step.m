function plot_first_time_step(t, Q, mup, mu, Sp, S, X, X0, Xp, y)
    L = 5;
    
    plot_prior_belief(1, L, mu, S, X0);
    plot_prediction(2, L, mu, mup, S, Sp, Xp);
    plot_measurement(3, t, L, mu, mup, S, Sp, Q, X, y);
end

