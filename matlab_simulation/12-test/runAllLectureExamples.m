% Run all lecture examples

filelist = dir('../01-examples_lecture/**/*.m')

for i=1:length(filelist)
    filelist = dir('../01-examples_lecture/**/*.m');
    str = sprintf('Running %s',filelist(i).name);
    disp(str)
    run(filelist(i).name)
end
