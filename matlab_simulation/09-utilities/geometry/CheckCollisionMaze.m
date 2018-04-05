function [ collided ] = CheckCollisionMaze( s, f, map )
    % parametrize the lines
    step = f - s;
    
    for i=1:size(map,1)
        e1 = [map(i,1) map(i,3)];
        e2 = [map(i,2) map(i,4)];
        estep = e2 - e1;
        
        q_num = (s(2)-e1(2))*step(1) - (s(1)-e1(1))*step(2);
        q_den = estep(2)*step(1) - estep(1)*step(2);
        
        if(q_den == 0)
            %disp('q_den zero');
            continue;
        end
        
        q = q_num/q_den;
        
        if(q < 0 || q > 1)
            %disp('q not collided');
            continue;
        end
        
        t_num = (s(2)-e1(2))*estep(1) - (s(1)-e1(1))*estep(2);
        t_den = q_den;
        %t_num = e1(1) + q*estep(1) - s(1);
        t = t_num / t_den;
        
        if(t > 0 && t < 1)
            %disp('t collided');
            collided = 1;
            return;
        end
    end
    
    collided = 0;
end

