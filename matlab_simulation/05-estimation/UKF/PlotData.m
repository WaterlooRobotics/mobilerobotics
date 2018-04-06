function PlotData(t,x,mu_u,mu_S,mu_Su,mu,S,S_u)
%Plots the UKF,EKF,True state and ground
    clf; hold on;
    % True state
    plot(x(1,2:t),x(3,2:t), 'ro--')
    % EKF
    plot(mu_S(1,2:t),mu_S(3,2:t), 'bx--')
    % UKF
    plot(mu_Su(1,2:t),mu_Su(3,2:t), 'gx--')
    % EKF Ellipses
    mu_pos = [mu(1) mu(3)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    % UKF Ellipses
    mu_pos_u = [mu_u(1) mu_u(3)];
    S_pos_u = [S_u(1,1) S_u(1,3); S_u(3,1) S_u(3,3)];
    error_ellipse(S_pos_u,mu_pos_u,0.75);
    error_ellipse(S_pos_u,mu_pos_u,0.95);
    % Ground
    Ground(0,0)
    %Graph properties
    title('True state, EKF and UKF')
    axis equal
    axis([-1 20 -10 10])
    xlabel('ground distance');
    ylabel('height');
    legend('True state', 'EKF', 'UKF')
    F(t-1) = getframe;

end

