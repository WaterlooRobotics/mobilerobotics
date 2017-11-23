function CompareEKF_UKF(T,x,mu_S,mu_Su )
%The function plots the mu and sigma from EKF and UKF on the same graph to
%be compared
    clf; hold on;
    e = sqrt((x(1,2:end)-mu_S(1,2:end)).^2+(x(2,2:end)-mu_S(2,2:end)).^2);
    plot(T(2:end),e,'b', 'LineWidth', 1.5);
    eu = sqrt((x(1,2:end)-mu_Su(1,2:end)).^2+(x(2,2:end)-mu_Su(2,2:end)).^2);
    plot(T(2:end),eu,'g', 'LineWidth', 1.5);
    title('Position Estimation Errors for EKF and UKF')
    xlabel('Time (s)');
    ylabel('X-Z Position Error (m)');
    legend('EKF','UKF');
end

