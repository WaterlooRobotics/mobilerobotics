% RANSAC correspondence outlier rejection for measurement update
clear; clc; rng('default');

% Features
N = 300;
map = 100*rand(N,2);

% Robot parameters
x0 = [15 15 45*pi/180];
rmax = 50;
thmax = 60*pi/180;
Q = 0.0001;

mu0 = [10 20 90*pi/180];  %Initial estimate of pose

% Visible features measurement vector
meas_list = zeros(N,1);
j=1;
for i=1:N
    meas_list(i) = inview(map(i,:),x0,rmax,thmax);
    if (meas_list(i))
        d = sqrt(Q)*randn(1,1);
        meas_raw(j) = atan2(map(i,2)-x0(2),map(i,1)-x0(1))- x0(3)+d;
        j = j+1;
    end
end
meas_ind = find(meas_list);
meas_count = length(meas_ind);

% Plot of visible features
figure(1); clf; hold on;
plot (map(:,1), map (:,2), 'bo')
plot (map(meas_ind,1), map (meas_ind,2), 'ro')
plot (x0(1), x0(2), 'gx', 'MarkerSize', 10, 'LineWidth',2)
title('Map of features')
legend('All features', ' Visible features', 'Robot position')
axis([0 100 0 100])
axis equal

% Incorrect correspondence insertion 
outlier_ratio = 0.2;
outlier_count =  floor(outlier_ratio*meas_count);
outlier_ind = find(rand(outlier_count,1)>outlier_ratio);
meas_ind_old = meas_ind;
for i =1:outlier_count
    meas_ind(i) = ceil(meas_count*rand(1,1));
end

% Plot of outliers
figure(2); clf; hold on;
plot (map(:,1), map (:,2), 'bo')
plot (map(meas_ind,1), map (meas_ind,2), 'ro')
%plot (x0(1), x0(2), 'gx', 'MarkerSize', 10, 'LineWidth',2)
for i =1:outlier_count
    plot([map(meas_ind_old(i),1) map(meas_ind(i),1)],[map(meas_ind_old(i),2) map(meas_ind(i),2)], 'm')
end
title('Outliers included in dataset')
axis([0 100 0 100])
axis equal
    
% RANSAC
NLLS_iters = 50;
RANSAC_iters = 100;
sample = 5;
inlier_threshold = 0.03;
max_set_length = 0;
max_set = [];

for k = 1:RANSAC_iters
    % Initialize estimate
    mu = mu0';
    mu_s = zeros(3,NLLS_iters+1);
    mu_s(:,1) = mu;

    % Pick samples to use for solution
    sample_ind = ceil(meas_count*rand(sample,1));
    
    % Solve NLLS to find robot position
    meas_sample = meas_raw(sample_ind);
    for m=1:NLLS_iters
        H = [];
        for n = 1:sample
            cur_pt = map(meas_ind(sample_ind(n)),:);
            rp(n) = sqrt((cur_pt(1)-mu(1))^2+(cur_pt(2)-mu(2))^2);
            thetap(n) = atan2(cur_pt(2)-mu(2),cur_pt(1)-mu(1))-mu(3);
            H = [H; (cur_pt(2)-mu(2))/rp(n)^2 ...
                   -(cur_pt(1)-mu(1))/rp(n)^2 ...
                   -1];
        end
        
        mu = mu + H\(meas_sample'-thetap');
        mu_s(:,m+1) = mu;
    end

    % Find inlier set
    inlier_set = [];
    for p = 1:meas_count
        if (find(sample_ind == p))
            inlier_set = [inlier_set;p];
        else
            cur_pt = map(meas_ind(p),:);
            thetac =  atan2(cur_pt(2)-mu(2),cur_pt(1)-mu(1))-mu(3);
            error = abs(meas_raw(p)-thetac);
            if (error < inlier_threshold)
                inlier_set = [inlier_set;p];
            end            
        end
    end

    
    % Save biggest inlier set
    if (length(inlier_set)>max_set_length)
        max_set_length = length(inlier_set);
        max_set = inlier_set;
    end
    r_size(k) = length(inlier_set);
    r_error(k,:) = min(100,norm(mu-x0'));

    figure(3); clf; hold on;
    plot (map(:,1), map (:,2), 'bo')
    plot (map(meas_ind,1), map (meas_ind,2), 'ro')
    plot (x0(1), x0(2), 'gx', 'MarkerSize', 10, 'LineWidth',2)
    for i =1:length(sample_ind)
        plot (mu(1), mu(2), 'rx', 'MarkerSize', 10, 'LineWidth',2)
        plot([mu(1) mu(1)+100*cos(meas_sample(i)+mu(3))],[mu(2) mu(2)+100*sin(meas_sample(i)+mu(3))], 'c');
        plot( map(meas_ind(sample_ind),1),map(meas_ind(sample_ind),2), 'kx')
    end
    title('Inlier set measurements and resulting hypothesis')
    axis equal
    axis([0 100 0 100])


end

figure(4);clf;hold on;
plot(r_size,r_error,'bo');
axis([0 100 0 120])
title('Estimation error vs inlier set size')
ylabel('Norm of estimation error')
xlabel('Inlier set size')

% Resolve NLLS with biggest inlier set (iterate on measurement update)
meas_set = meas_raw(max_set);
for m=1:NLLS_iters
    H = [];
    for n = 1:max_set_length
        cur_pt = map(meas_ind(max_set(n)),:);
        rp(n) = sqrt((cur_pt(1)-mu(1))^2+(cur_pt(2)-mu(2))^2);
        thetap(n) = atan2(cur_pt(2)-mu(2),cur_pt(1)-mu(1))-mu(3);
        H = [H; (cur_pt(2)-mu(2))/rp(n)^2 ...
            -(cur_pt(1)-mu(1))/rp(n)^2 ...
            -1];
    end
    mu = mu + H\(meas_set'-thetap');
    mu_s(:,m+1) = mu;
end
% Plot NLLS result for current seed set
figure(5); clf; hold on;
plot (map(:,1), map (:,2), 'bo')
plot (map(meas_ind,1), map (meas_ind,2), 'ro')
plot (x0(1), x0(2), 'gx', 'MarkerSize', 10, 'LineWidth',2)
plot (mu_s(1,:),mu_s(2,:), 'mx-', 'LineWidth', 2, 'MarkerSize', 6);
for n = 1:max_set_length
    cur_pt = map(meas_ind(max_set(n)),:);
    plot(cur_pt(1), cur_pt(2), 'gx')    
end
axis([0 100 0 100])
axis equal
