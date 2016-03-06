function [mup,Spp,Hmu]=ekf_FS1(Xp,y,mu,S,Qi,check)
  if (check == 1) %nonobserved features are all 1;old features are 0;
                % Feature initialization
                mup(1,1) = Xp(1,1)+y(1,1)*cos(y(2,1)+Xp(3,1));
                mup(2,1) = Xp(2,1)+y(1,1)*sin(y(2,1)+Xp(3,1));
                % Predicted range
                dx = mup(1,1)-Xp(1,1);
                dy = mup(2,1)-Xp(2,1);
                rp = sqrt((dx)^2+(dy)^2);
                % Calculate Jacobian
                Ht = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];
                Spp= Ht\Qi*inv(Ht)';
                
            else
                % Measurement processing
                % Predicted range
                dx = mu(1,1)-Xp(1,1);
                dy = mu(2,1)-Xp(2,1);
                rp = sqrt((dx)^2+(dy)^2);
                % Calculate Jacobian
                Ht = [ dx/rp dy/rp; -dy/rp^2 dx/rp^2];
                % Calculate Innovation
                
                I = y(:,1)-[rp; mod(atan2(dy,dx)-Xp(3,1)+pi,2*pi)-pi];
                
                % Measurement update
                Q = Ht*S*Ht' + Qi;
                K = S*Ht'/Q;
                mup = mu + K*I;
                Spp = (eye(2)-K*Ht)*S;

            end
        Hmu=[rp; mod(atan2(dy,dx)-Xp(3,1)+pi,2*pi)-pi];
end