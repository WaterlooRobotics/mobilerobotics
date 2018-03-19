%% Wavefront planning in gallery
[boundary, obsts, tour] = makegallerymap();

% Convert obstacles to array
obs = [];
for i=1:length(obsts)
    obs = [obs; obsts{i}; obsts{i}(1,:); NaN NaN];
end

% Convert map to occupancy grid
M = max(boundary(:,1));
N = max(boundary(:,2));
map = ones(M,N);

for i=1:M
    for j = 1:N
        if (inpolygon(i,j,boundary(:,1), boundary(:,2)))
            map(i,j) = inpolygon(i,j,obs(:,1), obs(:,2));
        end
    end
end

%% Set up simulation

Tmax = 200;
startPos = tour(1,:);
endCount = length(tour(:,1));
curGoal = 1;
x = zeros(2,Tmax);
x(:,1) = startPos';
dx = 0.01;
t = 1;
gVcur = [1 1];
newplan = 1;
t_start = 1;

while ((t<Tmax))
    t=t+1 % Update time
    pos = x(:,t-1)'
    tour(curGoal,:)
    norm(pos-tour(curGoal,:))
    % Check if it's time to pause and change targets
    if (norm(pos-tour(curGoal,:))<1)
        % Pretend to pause and talk
        x(:,t:t+4) = x(:,t-1)*ones(1,5);
        t = t+5
 
        % Update goal unless at end
        if (curGoal==endCount)
            break;
        end

        % Compute path to next goal
        startPos = tour(curGoal,:);
        curGoal = curGoal+1;
        endPos = tour(curGoal,:);
        [wave,curpath] = wavefront(map,startPos,endPos, 'urdl');
        t_start=t-1;
    end
    x(:,t) = curpath(t-t_start,:)'
    figure(1);clf;hold on
    imagesc(wave'); 
    plot(x(1,t_start:t),x(2,t_start:t),'r')
    plot(tour(curGoal,1), tour(curGoal,2), 'go')
    pause(0.01);
end
 