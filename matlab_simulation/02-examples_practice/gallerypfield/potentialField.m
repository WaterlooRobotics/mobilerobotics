% Gallery map
[boundary, obsts, tour] = makegallerymap();

% Convert map to occupancy grid
M = max(boundary(:,1));
N = max(boundary(:,2));
map = ones(M,N);
numObsts = length(obsts);

for i=1:numObsts
    co=obsts{i};
    [co(:,1), co(:,2)] = poly2ccw(co(:,1), co(:,2)); % Convert to ccw poly
    K = convhull(co(:,1), co(:,2)); % Take convex hull
    co = co(K(1:end-1),:); 
    obsArea(i) = polyarea(co(:,1),co(:,2)); % Take area of convex hull
    co = [co; co(1,:)]; % Close polygon
    Cx=0;Cy=0; 
    for j = 1:length(co)-1
        Cx = Cx + (co(j,1)+co(j+1,1))*(co(j,1)*co(j+1,2)-co(j+1,1)*co(j,2));
        Cy = Cy + (co(j,2)+co(j+1,2))*(co(j,1)*co(j+1,2)-co(j+1,1)*co(j,2));
    end
    obsCentroid(i,:) = 1/6/obsArea(i)*[Cx Cy];
end
plot(obsCentroid(:,1), obsCentroid(:,2), 'mx') 


% Create potential field
Katt = 2; % Attractive
Krep1 = 10; % Repulsive
Krep2 = 500;%5000; % Repulsive
r0 = 0.5; % Radius of Repulsion
rc0 = 5;%10; %
Vmax = 1500; % Upper bound on potential
gVmax = 100;
gVmin = -100;

% Simulate a robot moving through the environment following steepest
% descent, recalculated for current position
Tmax = 20000;
startPos = tour(1,:);
endPos = tour(2,:);
curGoal = 2;
x = zeros(2,Tmax);
x(:,1) = startPos';
dx = 0.001;
t = 1;
gVcur = [1 1];

while ((t<Tmax))
    t=t+1;
    pos = x(:,t-1)';
    % Check if it's time to pause and change targets
    if (norm(pos-endPos)<1) || (norm(gVcur)< 1e-2)
        % Pretend to pause and talk
        % Update goal
        if (curGoal==length(tour(:,1)))
            break;
        else
            startPos = tour(curGoal,:);
            curGoal = curGoal+1;
            endPos = tour(curGoal,:);
        end
    end
    
    gVcur = Katt*(pos-endPos);
    for m=1:numObsts
        curobs = obsts{m};
        if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
            gVcur = [NaN NaN];
        else
            curpoly = [curobs curobs([2:end, 1],:)];
            [minD,minPt, d, pt, ind] = minDistToEdges(pos, curpoly);
            if (minD < r0)
                gVcur = gVcur + Krep1*(-1/minD+1/r0)*(pos-minPt)/minD^(3);
            end
            centD = norm(pos-obsCentroid(m,:));
            if (centD < rc0)
                gVcur = gVcur + Krep2*(-1/centD+1/rc0)*(pos-obsCentroid(m,:))/centD^(3);
            end
        end
    end
    x(:,t) = x(:,t-1) -dx.*gVcur';
 
end
figure(1); hold on;
plot(x(1,1:t-1), x(2,1:t-1));
figure(4);
plot(x(:,1:t-1)');
title('Robot Position');
legend('X', 'Y', 'Location', 'South');
axis([0 t-1 0 N]);

% Grid up the space
dx = 0.4;
dy = 0.4;
[X,Y] = meshgrid([0:dx:30],[0:dy:30]);
startPos = tour(1,:);
endPos = tour(4,:);

disp('Tour complete, making PField')
% Calculate potential field at each grid point
V = zeros(size(X));
[n,m] = size(X);
gV = zeros(2,n,m);
for i=1:length(X(:,1))
    for j=1:length(Y(1,:))
        % Current robot position
        pos = [X(i,j) Y(i,j)];
        % Attractive Potential
        V(i,j) = 1/2*Katt*norm(pos-endPos)^2; % potential
        gV(:,i,j) = Katt*(pos-endPos);  % gradient
        
        %Repulsive potentials
        for m=1:numObsts
            curobs = obsts{m};
            if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
                V(i,j) = Vmax;
                gV(:,i,j) = [NaN NaN];
            else
                % Find potential based on minimum distance to obstacle
                curpoly = [curobs curobs([2:end, 1],:)];
                [minD,minPt, d, pt, ind] = minDistToEdges(pos, curpoly);
                if (minD < r0)
                    V(i,j) = V(i,j) + 1/2*Krep1*(1/minD-1/r0)^2;
                    gV(:,i,j) = gV(:,i,j) + Krep1*(-1/minD+1/r0)*(pos'-minPt')/minD^(3);
                end
                % Add potential of distance to center, to avoid getting
                % stuck on flat walls
                centD = norm(pos-obsCentroid(m,:));
                if (centD < rc0)
                    V(i,j) =  V(i,j) + 1/2*Krep2*(1/centD-1/rc0)^2;
                    gV(:,i,j) = gV(:,i,j) + Krep2*(-1/centD+1/rc0)*(pos'-obsCentroid(m,:)')/centD^(3);
                end
            end
        end
        V(i,j) = max(0,min(V(i,j),Vmax));
        gV(1,i,j) = max(gVmin,min(gV(1,i,j),gVmax));
        gV(2,i,j) = max(gVmin,min(gV(2,i,j),gVmax));
    end
end

figure(2); clf; hold on;
surf(X,Y,V)
title('Potential Field Surface');
view(15, 80);
%axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2) 0 Vmax])

figure(3); clf; hold on;
quiver(X,Y,squeeze(-gV(1,:,:)),squeeze(-gV(2,:,:)))
title('Potential Field');
axis([0 M 0 N]);
%plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos);

