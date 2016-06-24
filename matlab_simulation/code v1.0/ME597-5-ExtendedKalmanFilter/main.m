% Mulitirate EKF example
close all;
clear;
clc;
rng('shuffle');

makemovie = 1;
if(makemovie)
vidObj = VideoWriter('ekf.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 8;
open(vidObj);
end

% robots values
r =0.25;
l =0.3;

% Speed and frequency
w1=-15.5;
w2=10.5;
w3=1.5;
f=10;
W=[w1 w2 w3]';

% Inital state and time in sec
Xint = [0 0 0.1]';
t=15;
dt=1/f;


% Covariance matrix
e =diag(sqrt([0.05 0.05 toRad(0.5)^2]));
R = [0.01 0 0; 0 0.01 0; 0 0 0.01;];
Q = [0.4 0 0; 0 0.4 0; 0 0 0.4;];
Sk = [1 0 0; 0 1 0; 0 0 0.4;];

% Correction matrix Cm is 2xn where n dim of observing messurements
Cm=[10 10 1; 0.01 0.01 0.4];

% Multi rate EKF
for i=2:t*f

% World frame motion
[Xideal(:,i),X(:,i),Y,V]=omniRobotFrame(r,l,W,Xint,dt,e);


% Update initial state
Xint=Xideal(:,i);

[Xk(:,i),Sk]=MultiRateEKFGit(Y,V,f,e,Xint,R,Q,Sk,i,Cm);

    mup_S(:,i) = Xideal(:,i);
    mu_S(:,t) = Xk(:,i);

figure(1);clf; hold on;

    plot(X(1,2:i),X(2,2:i), 'ro--')
    plot(Xideal(1,2:i),Xideal(2,2:i), 'bx--')
    mu_pos = [Xk(1,i) Xk(2,i)];
    S_pos = [Sk(1,1) Sk(1,2); Sk(2,1) Sk(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
     error_ellipse(S_pos,mu_pos,0.95);
    title('True state and belief')
     axis equal
    axis([-8 8 -14 2])
    legend('True trajectyory', 'Multirate EKF trajectory');
    if (makemovie) writeVideo(vidObj, getframe(gca)); end

end

figure
plot(Xideal(1,:),Xideal(2,:),'b-',X(1,:),X(2,:),'r-',Xk(1,:),Xk(2,:),'g-');
title('Ideal,True and Estimated position Multi rate EKF');
if (makemovie) writeVideo(vidObj, getframe(gca)); end


if (makemovie) close(vidObj); end