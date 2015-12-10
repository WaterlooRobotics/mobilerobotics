%% Path setup for ME 597 Code library
% Sets up the path for all code in the ME 597 library, and saves the new
% path to automatically reload on Matlab restart.  Run once.
% If you add directories to the library, you can also add them to the
% dirlist if you like.
%
% Tested in Windows, should work in Linux/Mac

% Find base directory
base = pwd;
cd(base);

% Set slash convention
os = getenv('OS');
sep = '/';
if (strcmp(os, 'Windows_NT'))
    sep = '\';
end
   
% Add base and all subdirectories to path
addpath(base);

dirlist = {'3-motion','4-sensor','5-estimation','6-mapping','7-control','8-planning','9-utilities', '9-utilities/geometry', '10-environments'}; 

for i=1:length(dirlist)
    subdir = sprintf('%s%s%s', base,sep,dirlist{i});
    addpath(subdir);
end

savepath
