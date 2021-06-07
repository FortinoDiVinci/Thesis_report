function out = out2(fun) % gets the second output argument of anonymous function fun
    [~,out] = fun(); % fun must have no input arguments
end