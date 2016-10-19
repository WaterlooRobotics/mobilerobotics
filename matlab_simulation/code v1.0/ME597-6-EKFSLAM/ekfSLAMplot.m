function ekfSLAMplot(map,y,xr,mu_S,S,t,newfeature)
% This function is used for plotting in ekfSLAM
    
    M=length(map(1,:));    % size of map
    N=length(xr(:,1))+2*M; % number of possible states

    subplot(1,2,1); hold on;
    plot(map(1,:),map(2,:),'go', 'MarkerSize',10,'LineWidth',2);
    plot(xr(1,1:t),xr(2,1:t), 'ro--')
    plot([xr(1,t) xr(1,t)+1*cos(xr(3,t))],[xr(2,t) xr(2,t)+1*sin(xr(3,t))], 'r-')
    plot(mu_S(1,1:t),mu_S(2,1:t), 'bx--')
    plot([mu_S(1,t) mu_S(1,t)+1*cos(mu_S(3,t))],[mu_S(2,t) mu_S(2,t)+1*sin(mu_S(3,t))], 'b-')
    mu_pos = [mu_S(1,t),mu_S(2,t)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);

    for i=1:M
          if (~newfeature(i))
              fi = 2*(i-1)+1;
              fj = 2*i;
              plot([xr(1,t) xr(1,t)+y(fi,t)*cos(y(fj,t)+xr(3,t))], [xr(2,t) xr(2,t)+y(fi,t)*sin(y(fj,t)+xr(3,t))], 'c');
              plot(mu_S(3+fi,t),mu_S(3+fj,t), 'gx')
              mu_pos = [mu_S(3+fi,t) mu_S(3+fj,t)];
              S_pos = [S(3+fi,3+fi) S(3+fi,3+fj); S(3+fj,3+fi) S(3+fj,3+fj)];
              error_ellipse(S_pos,mu_pos,0.75);
          end
    end
    axis equal
    title('SLAM with Range & Bearing Measurements')
    
    % Plot Covariance
    subplot(1,2,2);
    image(10000*S);
    colormap('gray');
    axis('square')
    axis([0,N,0,N])
    title('Covariance matrix')
end
