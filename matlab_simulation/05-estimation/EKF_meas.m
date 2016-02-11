function mu_out = EKF_meas(mu,S,y,Q)
i=1;
dx = mu(3+2*(i-1)+1)-mu(1);
dy = mu(3+2*i)-mu(2);
rp = sqrt((dx)^2+(dy)^2);

Fi = zeros(5,length(mu));
Fi(1:3,1:3) = eye(3);
Fi(4:5,3+2*(i-1)+1:3+2*i) = eye(2);
Ht = [ -dx/rp -dy/rp 0 dx/rp dy/rp;
    dy/rp^2 -dx/rp^2 -1 -dy/rp^2 dx/rp^2]*Fi;
 
I = y(2*(i-1)+1:2*i)-[rp;(atan2(dy,dx) - mu(3))];
I(2) = mod(I(2)+pi,2*pi)-pi;
 
% Measurement update
K = S*Ht'*inv(Ht*S*Ht'+Q);
mu_out = mu + K*I;
