function index = input_check(index)
% User input check
% Input index should be between 1-7
if index<1 || index>7
    disp('Please choose a proper input trajectory (1-7).')
    disp('What is the input trajectory?')
    disp('1: Spiral; 2: Nudges; 3: Swerves; 4: Corner;')
    disp('5: A single lane change')
    disp('6: Get around an obstacle')
    disp('7: User defined motion model')
    prompt = '';
    index = input(prompt);
end
end
