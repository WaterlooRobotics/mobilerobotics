function [map newfeature y flist]=rearrangeMap(map,newfeature,y,flist,i,count)
% Function to reorganize the map, feature list, and measurements as new
% feature is observed.
% The newly discovered feature will be placed at the beginning of map,
% newfeature, y (measurements), and flist
% i: index for the newly observed feature in the feature list
% count: total number of features that have been detected


   map=[map(:,i),map(:,1:(i-1)),map(:,(i+1):end)];
   y=[y((2*i-1):2*i,:);y(1:2*(i-1),:);y((2*i+1):end,:)];
   newfeature=[newfeature(i);newfeature(1:(i-1));newfeature((i+1):end)];
   flist=[flist(i);flist(1:(i-1));flist((i+1):end)];
%    mu_S=[mu_S(1:3,t);mu_S((3+2*i-1):(3+2*i),t)];
%    mu_xr=mu_S(1:3,:);
%    mu_map=mu_S(4:end,:);
%    mu_map=[mu_map((2*i-1):2*i,:);mu_map(1:2*(i-1),:);mu_map((2*i+1):end,:)];
%    mu_S=[mu_xr;mu_map]

end
