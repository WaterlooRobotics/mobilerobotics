%this function will plot a list of trapezoids and label them with their index.
function plotTrapezoids(traps)

colours = [0.8 1 0.8;1 1 0.8;0.8 0.8 1 ];
hold on
for m = 1:1:length(traps)
    if(~traps(m).valid)
        continue;
    end
    patch(traps(m).pts(:,1),traps(m).pts(:,2), colours(mod(m,3)+1,:) ,'linestyle','-','EdgeColor',[0.3 0.6 0.3]);%,'facecolor','none'
    %text(mean(traps(m).pts(:,1)), mean(traps(m).pts(:,2)),sprintf('%d',m));
end
hold off
% axis equal
