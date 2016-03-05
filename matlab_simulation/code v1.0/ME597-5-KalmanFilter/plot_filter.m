function plot_filter(x,y,u,t,mu_S,mup_S,mu,S,example,makemovie,vidObj)

% This function plots each time step

if example == 1
    figure(1);clf;hold on;
    plot(T,x,'b')
    plot(T,y,'rx')
    plot(T,mup_S,'c--')
    plot(T,mu_S,'r--')
    plot(T,u,'g');
    plot(T,2*ones(size(T)),'m--');
    plot(T,10*ones(size(T)),'m--');
    title('State and estimates')
    legend('State','Measurement', 'Prediction', 'Estimate', 'Input')
end

if example == 2
    % Plot results
    figure(1);clf; hold on;
    plot(x(3,2:t),x(1,2:t), 'ro--')
    plot(y(2,2:t),y(1,2:t), 'gx')
    %plot(mup_S(3,1:t),mup_S(1,1:t), 'mx--')
    plot(mu_S(3,2:t),mu_S(1,2:t), 'bx--')
    mu_pos = [mu(3) mu(1)];
    S_pos = [S(3,3) S(3,1); S(1,3) S(1,1)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and beliefs')
    legend('State', 'Measurement','Estimate')
    axis equal
    axis([-10 10 -5 15])
end

if example == 3
    figure(1);clf; hold on;
    plot(x(3,2:t),x(1,2:t), 'ro--')
    if (mod(t,10)==0) 
        plot(y(3,t),y(1,t), 'gx'); 
    end
    %plot(mup_S(3,1:t),mup_S(1,1:t), 'mx--')
    plot(mu_S(3,2:t),mu_S(1,2:t), 'bx--')
    mu_pos = [mu(3) mu(1)];
    S_pos = [S(3,3) S(3,1); S(1,3) S(1,1)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and beliefs')
    legend('State', 'Measurement','Estimate')
    axis([-.5 2.5 -.5 2])
    if (makemovie) 
        writeVideo(vidObj, getframe(gca));
    end
end
