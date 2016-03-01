function plot_ellipse(mu, S)
    mu_pos = [mu(1) mu(3)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos, mu_pos, 0.75);
    error_ellipse(S_pos, mu_pos, 0.95);
end

