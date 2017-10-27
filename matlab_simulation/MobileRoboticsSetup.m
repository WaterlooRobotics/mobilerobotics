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
    rmpath(genpath(base));
    !rm pathdef.m
    disp('Removed all directories from path');
else
    addpath(genpath(base));
    % remove code v1.0 to avoid duplication, can be eliminated when v2.0
    % complete
    oldbase = strcat(base,sep,'code v1.0');
    rmpath(genpath(oldbase));
    savepath pathdef.m
    disp('Added all directories to path');
end

disp('Setup complete')
