function index = input_check(index)
% User input check
% Input index should be between 1-5
if index<1 || index>5
    disp('Please choose a proper input trajectory (1-5).')
    disp('What is the input trajectory?')
    disp('1: Spiral; 2: Nudges; 3: Swerves; 4: Corner;')
    disp('5: User defined motion model')
    prompt = '';
    index = input(prompt);
end
end