classdef Plotter
    methods
    end
    methods (Static)
        function plot_ekf_ukf_particle(figure_index, T, x, mu_S, mu_Su, muP_S)
            figure(figure_index);
            clf; 
            hold on;

            % EKF
            e = sqrt((x(1,2:end) - mu_S(1,2:end)).^2 + (x(3,2:end) - mu_S(3,2:end)).^2);
            plot(T(2:end),e,'b', 'LineWidth', 1.5);

            % UKF
            eu = sqrt((x(1,2:end) - mu_Su(1,2:end)).^2 + (x(3,2:end) - mu_Su(3,2:end)).^2);
            plot(T(2:end),eu,'g', 'LineWidth', 1.5);

            % PF
            ep = sqrt((x(1,2:end)- muP_S(1,2:end)).^2 + (x(3,2:end) - muP_S(3,2:end)).^2);
            plot(T(2:end),ep,'m', 'LineWidth', 1.5);

            % plot details
            title('Position Estimation Errors for EKF, UKF and PF')
            xlabel('Time (s)');
            ylabel('X-Z Position Error (m)');
            legend('EKF', 'UKF', 'Particle');
        end
    end
end