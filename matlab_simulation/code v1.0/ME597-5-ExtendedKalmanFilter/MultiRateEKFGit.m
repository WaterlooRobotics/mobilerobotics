function [Xk,Sk]=MultiRateEKFGit(Y,V,f,e,X0,R,Q,Sk,n,Cm)

dt = 1/f;


  

   % Predict state

   A=LinearOmniMotion(Y,V,X0(3),dt);
   S=A*Sk*A'+R;
   % Measurement
   [Z,h,H]=omniLinearMeasure(X0,Q);

   
   % Update State

   % Correction
  d=size(Cm);
  for j=1:d(2);
   if(mod(n,Cm(1,j))==0)
       Q1=Q;
       Q1(j,j)=Cm(2,j);
       [Zt,ht,H]=omniLinearMeasure(X0,Q1);
       Z(j,1)=Zt(j,1);
   end
  end
  
  % Kalman gain
    K= S*H'*inv(H*S*H'+Q);

   % Generate measurement
    Xk=X0+K*(Z-h);
    Sk=(eye(3)-K*H)*S;

   

end
