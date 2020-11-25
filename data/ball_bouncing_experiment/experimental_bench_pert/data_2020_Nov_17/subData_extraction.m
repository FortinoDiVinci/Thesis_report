clear all
close all

%T = readtable('data_2020_Nov_17_concatenated.csv');
T = readtable('data_vfo_3_phases_concatenated.csv');

Time = T.Time;
subDataTest = [];
subData_size = 1500; % ms
j = 1;

for it = 1:floor(length(T.Position)/subData_size)
    
    idx = (1:1500)+(it-1)*subData_size;
    
    % Avoid mixing two different trials in a single chunck
    if(any(diff(T.TrialNb(idx))))
        continue
    end
    
    % Avoid using cases where the ball was not moving yet
    if(all(~diff(T.Ball(idx))))
        continue
    end
    
    subDataTest(j).position = T.Position(idx);
    subDataTest(j).force = T.Force(idx);
    subDataTest(j).ball = T.Ball(idx);
    if T.Hand(idx(1))
        subDataTest(j).hand = 'right';
    else
        subDataTest(j).hand = 'left';
    end
    subDataTest(j).isBallImpact = T.IsBallImpact(idx(1));
    
    j = j + 1;
    
end

%save('youBotRhythmicTaskSubTrajectories.mat','subData')
save('data_vfo_3_phases_subData.mat','subDataTest')

rand_idx = randperm(length(subDataTest));

%% Disp

figure
for it = 1:9
    idx = rand_idx(it);
    subplot(3,3,it)
    plot(subDataTest(idx).position)
    hold on
    plot(subDataTest(idx).ball)
    yyaxis right
    plot(subDataTest(idx).force)
    if contains(subDataTest(idx).hand, "right")
        hand = "right hand.";
    else
        hand = "left hand.";
    end
    if subDataTest(idx).isBallImpact
        b_i = ", with";
    else
        b_i = ", without";
    end
    title("Trial n" + num2str(idx) + b_i + " ball haptic feedback, using the " + hand)
end