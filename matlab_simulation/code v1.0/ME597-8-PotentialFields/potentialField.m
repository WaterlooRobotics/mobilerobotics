% Potential field example
clear all; close all; clc

% Vehicle start and end position
startPos = [21.5 18.5];
endPos = [6 5];

% Set up environment

% Region Bounds
posMinBound = [-2 -2];
posMaxBound = [25 20];

% Number of obstacles
numObsts = 6;
% Size bounds on obstacles
minLen.a = 1;
maxLen.a = 3;
minLen.b = 2;
maxLen.b = 6;

% Random environment generation
obstBuffer = 0.5;
maxCount = 10000;
seedNumber = rand('state');
[aObsts,bObsts,obsPtsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, numObsts, startPos, endPos, obstBuffer, maxCount);
for i=1:numObsts
    obsCentroid(i,:) = (obsPtsStore(1,2*(i-1)+1:2*i)+obsPtsStore(3,2*(i-1)+1:2*i))/2;
end

% Plot random environment
figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos);
hold off

% Create potential field
Katt = 0.2; % Attractive
Krep = 1000; % Repulsive
r0 = 0.5; % Radius of Repulsion
rc0 = 4; %
Vmax = 50; % Upper bound on potential
gVmax = 10;
gVmin = -10;
% Grid up the space
dx =.4;
dy = .4;
[X,Y] = meshgrid([posMinBound(1):dx:posMaxBound(1)],[posMinBound(2):dy:posMaxBound(2)]);

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
            curobs = obsPtsStore(:,2*(m-1)+1:2*m);
            if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
                V(i,j) = Vmax;
                gV(:,i,j) = [NaN NaN];
            else
                % Find potential based on minimum distance to obstacle
                curpoly = [curobs curobs([2:end, 1],:)];
                [minD,minPt, d, pt, ind] = minDistToEdges(pos, curpoly);
                if (minD < r0)
                    V(i,j) = V(i,j) + 1/2*Krep*(1/minD-1/r0)^2;
                    gV(:,i,j) = gV(:,i,j) + Krep*(-1/minD+1/r0)*(pos'-minPt')/minD^(3);
                end
                % Add potential of distance to center, to avoid getting
                % stuck on flat walls
                centD = norm(pos-obsCentroid(m,:));
                if (centD < rc0)
                    V(i,j) =  V(i,j) + 1/2*Krep*(1/centD-1/rc0)^2;
                    gV(:,i,j) = gV(:,i,j) + Krep*(-1/centD+1/rc0)*(pos'-obsCentroid(m,:)')/centD^(3);
                end
            end
        end
        V(i,j) = max(0,min(V(i,j),Vmax));
        gV(1,i,j) = max(gVmin,min(gV(1,i,j),gVmax));
        gV(2,i,j) = max(gVmin,min(gV(2,i,j),gVmax));
    end
    i
end

figure(2); clf; hold on;
surf(X,Y,V)
axis([posMinBound(1) posMaxBound(1) posMinBound(2) posMaxBound(2) 0 Vmax])

figure(3); clf; hold on;
quiver(X,Y,squeeze(-gV(1,:,:)),squeeze(-gV(2,:,:)))
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos);

% Simulate a robot moving through the environment following steepest
% descent, recalculated for current position
Tmax = 10000;
x = zeros(2,Tmax);
x(:,1) = startPos';
dx = 0.01;
t = 1;
gVcur = [1 1];
while ((norm(gVcur)>0.01) && (t<Tmax))
    t=t+1;
    pos = x(:,t-1)';
    gVcur = Katt*(pos-endPos);
    for m=1:numObsts
        curobs = obsPtsStore(:,2*(m-1)+1:2*m);
        if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
            gVcur = [NaN NaN];
        else
            curpoly = [curobs curobs([2:end, 1],:)];
            [minD,minPt, d, pt, ind] = minDistToEdges(pos, curpoly);
            if (minD < r0)
                gVcur = gVcur + Krep*(-1/minD+1/r0)*(pos-minPt)/minD^(3);
            end
            centD = norm(pos-obsCentroid(m,:));
            if (centD < rc0)
                gVcur = gVcur + Krep*(-1/centD+1/rc0)*(pos-obsCentroid(m,:))/centD^(3);
            end
        end
    end
    x(:,t) = x(:,t-1) -dx.*gVcur';
 
end
figure(1); hold on;
plot(x(1,1:t), x(2,1:t));
figure(4);
plot(x(:,1:t)');