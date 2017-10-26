% resampling parameters
M = 200;
X = rand(M, 2);
W = [1:M] ./ M;

for t = 1:200
    % particle filter estimation
    for m = 1:M
        seed = rand(1);
        X(m, :, t + 1) = X(find(W > seed, 1), :, t);
    end
end


% plot resampling of particles
t_range = [1, 2, 5, 10, 20, 30, 50, 70, 200];
figure(1);
clf;

for plot_index = 1:length(t_range)
    t = t_range(plot_index);
    
    subplot(3, 3, plot_index);
    plot(X(:, 1, t), X(:, 2, t),'ro');
    axis([0 1 0 1]);
    xlabel(['t = ' num2str(t)]);
end
