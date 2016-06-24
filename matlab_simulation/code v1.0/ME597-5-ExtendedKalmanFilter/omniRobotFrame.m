function [Xideal,X,Y,V]=omniRobotFrame(r,l,W,Xinit,dt,e)

Y=[ 0 -r*sqrt(3)/2 r*sqrt(3)/2 ;...
      r -r/2 -r/2;...
      r/(3*l) r/(3*l) r/(3*l);]*W;
   Wf=worldFrame(Xinit(3));
   V=Wf*[sqrt(Y(1)^2+Y(2)^2)*dt sqrt(Y(1)^2+Y(2)^2)*dt Y(3)*dt]';
    Xideal=Xinit+V;
    N=Noise(3,e)
    X=Xideal+N;
end