function MobileRoboticsSetup(varargin);
%% Path setup for ME 597 Code library
% Sets up the path for all code in the ME 597 library, and saves the new
% path to automatically reload on Matlab restart.  Run once.
% If you add directories to the library, you can also add them to the
% dirlist if you like.
%
% To remove installed directories from path, use
% MobileRoboticsSetup('remove');
%
% Tested in Windows, should work in Linux/Mac
if (nargin == 1)
    mode = varargin{1};
else
    mode = 'setup';
end

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
if (strcmp(mode, 'remove'))
    rmpath(base);
else
    addpath(base);
end

dirlist = {'03-motion','04-sensor','05-estimation','06-mapping','07-control','08-planning','09-utilities', '09-utilities/geometry', '10-environments'}; 

for i=1:length(dirlist)
    subdir = sprintf('%s%s%s', base,sep,dirlist{i});
    if (strcmp(mode, 'remove'))
        rmpath(subdir);
    else
        addpath(subdir);
    end
end

savepath
disp('Setup complete')
if (strcmp(mode, 'remove'))
    disp('Removed all directories from path');
else
    disp('Added all directories to path');
end
