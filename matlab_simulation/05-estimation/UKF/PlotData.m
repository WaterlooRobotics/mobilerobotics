function PlotData(t,x,mu_u,mu_S,mu_Su,mu,S,S_u)
%Plots the UKF,EKF,True state and ground
    clf; hold on;
    % True state
    plot(x(1,2:t),x(2,2:t), 'ro--')
    % EKF
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    % UKF
    plot(mu_Su(1,2:t),mu_Su(2,2:t), 'gx--')
    % EKF Ellipses
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    % UKF Ellipses
    mu_pos_u = [mu_u(1) mu_u(2)];
    S_pos_u = [S_u(1,1) S_u(1,2); S_u(2,1) S_u(2,2)];
    error_ellipse(S_pos_u,mu_pos_u,0.75);
    error_ellipse(S_pos_u,mu_pos_u,0.95);
    % Ground
    Ground(0,0)
    %Graph properties
    title('True state, EKF and UKF')
    axis([-5 20 -1 10])
    legend('True state', 'EKF', 'UKF')
    F(t-1) = getframe;

end

